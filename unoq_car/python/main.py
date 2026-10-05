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
import threading
import time

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
#   error <= STOP_BAND                 -> stop, the minifig is centred
#   centred and error <= RESTART_BAND  -> stay stopped (hysteresis: detection jitter of
#                                         ~0.01 must not make the car twitch)
#   error <= PULSE_ZONE                -> creep in short pulses: drive PULSE_ON seconds, stop,
#                                         wait for a camera frame taken after the car stopped
#   error > PULSE_ZONE                 -> drive continuously, faster the further away
STOP_BAND = 0.02
RESTART_BAND = 0.05
PULSE_ZONE = 0.15
FULL_SPEED_ERROR = 0.30  # at this error or more, drive at MAX_PWM

MIN_PWM = 80             # slowest PWM (0-255) that still moves the car; raise it if it stalls
MAX_PWM = 170            # fastest PWM; lower it if the car overshoots into the pulse zone
MAX_STEP = 50            # max PWM change per control tick, so the car doesn't jerk

PULSE_PWM = 110          # PWM during a pulse; raise it if pulses don't move the car at all
PULSE_ON = 0.08          # seconds of driving per pulse; lower it for smaller steps
PULSE_SETTLE = 0.30      # seconds to wait after a pulse so the camera/YOLO/MQTT catch up

LOST_TIMEOUT = 0.5       # stop if no minifig has been seen for this many seconds
LOOP_PERIOD = 0.05       # control tick; the sketch stops itself after ~400 ms of silence

_lock = threading.Lock()
_latest = {"position": None, "received": 0.0}
_current = 0               # last commanded PWM, for ramping
_last_status = None
_centred = False           # stopped inside STOP_BAND; stay stopped until RESTART_BAND is left
_pulsing = False           # in the pulse zone (vs. driving continuously)
_pulse_speed = 0           # signed PWM of the current pulse
_pulse_until = 0.0         # monotonic time the current pulse ends
_settle_until = 0.0        # no new pulse before this time


def direction(error: float) -> int:
    """+1 (forward) or -1 (backward): the way to drive to move the minifig towards TARGET."""
    # The minifig is past the centre in the + direction, so move it in the - direction.
    away = 1 if error > 0 else -1
    return -away if FORWARD_INCREASES else away


def drive_speed(error: float) -> int:
    """Signed PWM for continuous driving: MIN_PWM at PULSE_ZONE, up to MAX_PWM at FULL_SPEED_ERROR."""
    fraction = min(1.0, max(0.0, (abs(error) - PULSE_ZONE) / (FULL_SPEED_ERROR - PULSE_ZONE)))
    return direction(error) * round(MIN_PWM + fraction * (MAX_PWM - MIN_PWM))


def plan(position, received: float, now: float):
    """Decide this tick's motor speed. Returns (speed, ramped, status)."""
    global _centred, _pulsing, _pulse_speed, _pulse_until, _settle_until

    if position is None or now - received > LOST_TIMEOUT:
        _centred = _pulsing = False
        return 0, False, "no minifig seen - stopped"

    error = position - TARGET
    magnitude = abs(error)

    if magnitude <= STOP_BAND or (_centred and magnitude <= RESTART_BAND):
        _centred, _pulsing = True, False
        return 0, False, "minifig centred - stopped"
    _centred = False

    if magnitude > PULSE_ZONE:
        _pulsing = False
        speed = drive_speed(error)
        return speed, True, "driving forward" if speed > 0 else "driving backward"

    if not _pulsing:
        # Just arrived from continuous driving (or from centred): stop and let the car and
        # the camera settle before judging where the minifig really is.
        _pulsing = True
        _pulse_until = now
        _settle_until = now + PULSE_SETTLE
        return 0, False, "pulsing - settling"
    if now < _pulse_until:
        return _pulse_speed, False, "pulsing - moving"
    if now < _settle_until or received < _settle_until:
        # Wait until the settle time is over AND a position has arrived after it.
        return 0, False, "pulsing - settling"

    _pulse_speed = direction(error) * PULSE_PWM
    _pulse_until = now + PULSE_ON
    _settle_until = _pulse_until + PULSE_SETTLE
    return _pulse_speed, False, "pulsing - moving"


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
        position = None
        if detections:
            best = max(detections, key=lambda d: d["confidence"])
            position = float(best["center_norm"][AXIS])
    except (ValueError, KeyError, TypeError, AttributeError) as e:
        print(f"Ignoring bad message: {e}")
        return

    with _lock:
        _latest.update(position=position, received=time.monotonic())


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


def report(status: str, position=None):
    global _last_status
    if status != _last_status:
        where = "" if position is None else f" (minifig at {AXIS}={position:.2f})"
        print(f"{status}{where}")
        _last_status = status


def loop():
    global _current

    with _lock:
        position, received = _latest["position"], _latest["received"]

    target, ramped, status = plan(position, received, time.monotonic())
    if status.startswith("no minifig"):
        position = None
    # Pulses must start at full PULSE_PWM straight away, so only continuous driving is ramped.
    speed = ramp(_current, target) if ramped else target

    # Sent every tick, even when stopped, so the sketch's watchdog knows we're alive.
    # Both motors get the same speed so the car drives straight.
    try:
        Bridge.call("set_motors", speed, speed)
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
