"""Green Minifig Finder car - Linux side of the Arduino UNO Q App Lab app.

Subscribes to the minifig positions published by minifig_mqtt.py (JSON over MQTT) and
drives a two-motor car so the green minifig ends up in the centre of the camera frame.

Both motors always spin together, so the car only drives forward or backward (it never
steers). It follows one image axis (AXIS): if the minifig is on the far side of the centre,
the car drives one way, if it is on the near side it drives the other way. If the car drives
away from the centre instead of towards it, flip FORWARD_INCREASES.

The motor speeds are sent to the sketch (sketch/sketch.ino) over the Bridge.
"""

import json
import statistics
import threading
import time
from collections import deque

import paho.mqtt.client as mqtt
from arduino.app_utils import App, Bridge

# ---- MQTT -------------------------------------------------------------------------
MQTT_BROKER = "10.243.102.145"  # IP address of the computer running Mosquitto
MQTT_PORT = 1883
MQTT_USERNAME = ""               # leave blank if the broker accepts anyone
MQTT_PASSWORD = ""
MQTT_TOPIC = "minifig/detections"

# ---- Steering ---------------------------------------------------------------------
# Which image axis the car moves along: "x" (left/right in the frame) or "y" (up/down).
AXIS = "x"
# True if driving forward moves the minifig towards higher x/y (right/down in the frame).
FORWARD_INCREASES = True

TARGET = 0.5             # centre of the frame (normalised 0..1)

# How far off centre the minifig is decides how the car moves (fractions of the frame):
#   error > PULSE_ZONE                 -> drive continuously, faster the further away
#   error <= PULSE_ZONE                -> creep in short pulses: drive PULSE_ON seconds, stop,
#                                         wait PULSE_SETTLE, then judge the position from
#                                         SETTLED_FRAMES frames taken after the car stopped
#   settled error <= STOP_BAND         -> stop, the minifig is centred
#   centred and error <= RESTART_BAND  -> stay stopped (hysteresis, so detection jitter
#                                         doesn't make the car twitch)
# Positions are the median of recent frames, so a single jumpy YOLO box can't stop the car.
STOP_BAND = 0.02
RESTART_BAND = 0.03
PULSE_ZONE = 0.15
SMOOTH_WINDOW = 0.2      # seconds of frames to take the median of
SETTLED_FRAMES = 3       # frames after a pulse has settled needed to judge the position
FULL_SPEED_ERROR = 0.30  # at this error or more, drive at MAX_PWM

MIN_PWM = 80             # slowest PWM (0-255) that still moves the car; raise it if it stalls
MAX_PWM = 170            # fastest PWM; lower it if the car overshoots into the pulse zone
MAX_STEP = 50            # max PWM change per control tick, so the car doesn't jerk

PULSE_PWM = 110          # PWM of the first pulse; raise it if pulses don't move the car at all
# A motor needs more power to start than to keep turning, so the PWM adapts to the car:
PULSE_BOOST = 15         # a pulse moved the minifig < MIN_PULSE_MOVE -> next pulse +PULSE_BOOST PWM
MIN_PULSE_MOVE = 0.005   # a pulse moved it > MAX_PULSE_MOVE -> next pulse -PULSE_BOOST PWM
MAX_PULSE_MOVE = 0.03    # (between PULSE_PWM and MAX_PWM; otherwise the PWM that worked is kept)
STALL_TIME = 0.5         # driving continuously but the minifig hasn't moved MIN_PULSE_MOVE in
                         # this many seconds -> add PULSE_BOOST to the driving PWM
PULSE_ON = 0.08          # seconds of driving per pulse; lower it for smaller steps
PULSE_SETTLE = 0.30      # seconds to wait after a pulse so the camera/YOLO/MQTT catch up

LOST_TIMEOUT = 0.5       # stop if no minifig has been seen for this many seconds

MATRIX_COLS = 13         # the UNO Q LED matrix shows the minifig's position in the image
MATRIX_ROWS = 8          # as a dot, scaled from center_norm to these columns/rows
DOT_REFRESH = 1.0        # resend the dot at least this often (e.g. after the sketch restarts)
LOOP_PERIOD = 0.05       # control tick; the sketch stops itself after ~400 ms of silence

_lock = threading.Lock()
_latest = {"position": None, "received": 0.0, "dot": None}
_history = deque()         # (received, position) of recent frames with a minifig
_current = 0               # last commanded PWM, for ramping
_last_status = None
_centred = False           # stopped inside STOP_BAND; stay stopped until RESTART_BAND is left
_pulsing = False           # in the pulse zone (vs. driving continuously)
_pulse_speed = 0           # signed PWM of the current pulse
_pulse_until = 0.0         # monotonic time the current pulse ends
_settle_until = 0.0        # no new pulse before this time
_pulse_pwm = PULSE_PWM     # PWM of the next pulse (boosted while pulses don't move the car)
_last_judged = None        # settled position before the last pulse
_drive_boost = 0           # extra PWM for continuous driving while the car is stalled
_stall_check = None        # (time, position) continuous driving is compared against
_shown_dot = None          # (col, row) last sent to the LED matrix
_dot_sent = 0.0            # monotonic time it was sent


def direction(error: float) -> int:
    """+1 (forward) or -1 (backward): the way to drive to move the minifig towards TARGET."""
    # The minifig is past the centre in the + direction, so move it in the - direction.
    away = 1 if error > 0 else -1
    return -away if FORWARD_INCREASES else away


def drive_speed(error: float) -> int:
    """Signed PWM for continuous driving: MIN_PWM at PULSE_ZONE, up to MAX_PWM at FULL_SPEED_ERROR."""
    fraction = min(1.0, max(0.0, (abs(error) - PULSE_ZONE) / (FULL_SPEED_ERROR - PULSE_ZONE)))
    pwm = MIN_PWM + fraction * (MAX_PWM - MIN_PWM) + _drive_boost
    return direction(error) * round(min(MAX_PWM, pwm))


def plan(position, received: float, history, now: float):
    """Decide this tick's motor speed. Returns (speed, ramped, status, judged position)."""
    global _centred, _pulsing, _pulse_speed, _pulse_until, _settle_until, _pulse_pwm, _last_judged
    global _drive_boost, _stall_check

    if position is None or now - received > LOST_TIMEOUT:
        _centred = _pulsing = False
        _drive_boost, _stall_check = 0, None
        return 0, False, "no minifig seen - stopped", None

    recent = [p for r, p in history if r >= now - SMOOTH_WINDOW] or [position]
    smoothed = statistics.median(recent)
    error = smoothed - TARGET

    if _centred:
        if abs(error) <= RESTART_BAND:
            return 0, False, "minifig centred - stopped", smoothed
        _centred = False  # drifted out of the centre: pulse back in

    if abs(error) > PULSE_ZONE:
        _pulsing = False
        if _stall_check is None or abs(smoothed - _stall_check[1]) >= MIN_PULSE_MOVE:
            _stall_check = (now, smoothed)  # moving (or just started): reset the stall timer
        elif now - _stall_check[0] >= STALL_TIME:
            _drive_boost = min(MAX_PWM - MIN_PWM, _drive_boost + PULSE_BOOST)
            _stall_check = (now, smoothed)
            print(f"  stalled at {AXIS}={smoothed:.3f} -> driving boost +{_drive_boost} PWM")
        speed = drive_speed(error)
        return speed, True, "driving forward" if speed > 0 else "driving backward", smoothed

    if not _pulsing:
        # Just arrived from continuous driving (or from centred): stop and let the car and
        # the camera settle before judging where the minifig really is.
        _pulsing = True
        _pulse_until = now
        _settle_until = now + PULSE_SETTLE
        _last_judged = None
        _drive_boost, _stall_check = 0, None
        return 0, False, "pulsing - settling", smoothed
    if now < _pulse_until:
        return _pulse_speed, False, "pulsing - moving", smoothed

    # Only judge the position from frames that arrived after the car had stopped and the
    # camera/YOLO/MQTT pipeline had caught up.
    settled = [p for r, p in history if r >= _settle_until]
    if now < _settle_until or len(settled) < SETTLED_FRAMES:
        return 0, False, "pulsing - settling", smoothed
    settled_pos = statistics.median(settled[-SETTLED_FRAMES:])
    error = settled_pos - TARGET

    if abs(error) <= STOP_BAND:
        _centred, _pulsing = True, False
        print(f"  settled at {AXIS}={settled_pos:.3f} (off by {error:+.3f}) -> centred")
        return 0, False, "minifig centred - stopped", settled_pos

    if _last_judged is not None:
        moved = abs(settled_pos - _last_judged)
        if moved < MIN_PULSE_MOVE:
            _pulse_pwm = min(MAX_PWM, _pulse_pwm + PULSE_BOOST)  # car didn't move: push harder
        elif moved > MAX_PULSE_MOVE:
            _pulse_pwm = max(PULSE_PWM, _pulse_pwm - PULSE_BOOST)  # step too big: ease off
    _last_judged = settled_pos

    _pulse_speed = direction(error) * _pulse_pwm
    _pulse_until = now + PULSE_ON
    _settle_until = _pulse_until + PULSE_SETTLE
    print(f"  settled at {AXIS}={settled_pos:.3f} (off by {error:+.3f}) -> "
          f"pulse {'forward' if _pulse_speed > 0 else 'backward'} at PWM {_pulse_pwm}")
    return _pulse_speed, False, "pulsing - moving", settled_pos


def ramp(current: int, target: int) -> int:
    """Move `current` towards `target` by at most MAX_STEP (stopping is always immediate)."""
    if target == 0:
        return 0
    return current + max(-MAX_STEP, min(MAX_STEP, target - current))


def on_connect(client, userdata, flags, reason_code, properties=None):
    if getattr(reason_code, "is_failure", reason_code != 0):
        print(f"MQTT connection refused: {reason_code}")
        return
    print(f"Connected to MQTT broker {MQTT_BROKER}:{MQTT_PORT}, listening on '{MQTT_TOPIC}'")
    client.subscribe(MQTT_TOPIC)  # re-subscribe on every (re)connect


def on_message(client, userdata, msg):
    try:
        detections = json.loads(msg.payload).get("detections") or []
        position = dot = None
        if detections:
            best = max(detections, key=lambda d: d["confidence"])
            position = float(best["center_norm"][AXIS])
            dot = (float(best["center_norm"]["x"]), float(best["center_norm"]["y"]))
    except (ValueError, KeyError, TypeError, AttributeError) as e:
        print(f"Ignoring bad message: {e}")
        return

    now = time.monotonic()
    with _lock:
        _latest.update(position=position, received=now, dot=dot)
        if position is not None:
            _history.append((now, position))
        while _history and _history[0][0] < now - 1.0:
            _history.popleft()


def start_mqtt() -> mqtt.Client:
    try:  # paho-mqtt 2.x
        client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id="unoq-minifig-car")
    except AttributeError:  # paho-mqtt 1.x
        client = mqtt.Client(client_id="unoq-minifig-car")
    if MQTT_USERNAME:
        client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
    client.on_connect = on_connect
    client.on_message = on_message
    client.reconnect_delay_set(min_delay=1, max_delay=10)
    client.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=30)  # keeps retrying until the broker is up
    client.loop_start()
    return client


def to_matrix(dot):
    """Scale a (0..1, 0..1) image position to a (col, row) on the LED matrix; None -> (-1, -1)."""
    if dot is None:
        return -1, -1
    col = min(MATRIX_COLS - 1, max(0, round(dot[0] * (MATRIX_COLS - 1))))
    row = min(MATRIX_ROWS - 1, max(0, round(dot[1] * (MATRIX_ROWS - 1))))
    return col, row


def show_dot(dot, now: float):
    """Send the dot to the sketch when it moves (and every DOT_REFRESH seconds)."""
    global _shown_dot, _dot_sent
    cell = to_matrix(dot)
    if cell != _shown_dot or now - _dot_sent >= DOT_REFRESH:
        Bridge.call("set_dot", *cell)
        _shown_dot, _dot_sent = cell, now


def report(status: str, position=None):
    global _last_status
    if status != _last_status:
        where = "" if position is None else f" (minifig at {AXIS}={position:.2f})"
        print(f"{status}{where}")
        _last_status = status


def loop():
    global _current

    with _lock:
        position, received, dot = _latest["position"], _latest["received"], _latest["dot"]
        history = list(_history)

    now = time.monotonic()
    target, ramped, status, position = plan(position, received, history, now)
    if position is None:
        dot = None  # minifig lost: clear the dot too
    # Pulses must start at full PULSE_PWM straight away, so only continuous driving is ramped.
    speed = ramp(_current, target) if ramped else target

    # Sent every tick, even when stopped, so the sketch's watchdog knows we're alive.
    # Both motors get the same speed so the car drives straight.
    try:
        Bridge.call("set_motors", speed, speed)
        show_dot(dot, now)
    except (ValueError, TimeoutError) as e:
        # The sketch may not have registered set_motors yet right after startup.
        _current = 0
        report("waiting for sketch")
        print(f"Bridge error: {e}")
        time.sleep(0.5)
        return

    _current = speed
    report(status, position)
    time.sleep(LOOP_PERIOD)


start_mqtt()
App.run(user_loop=loop)
