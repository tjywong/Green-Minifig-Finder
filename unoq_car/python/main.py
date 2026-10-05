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
DEADBAND = 0.05          # an axis counts as centred within this fraction of the frame
FULL_SPEED_ERROR = 0.30  # at this error or more, the axis commands MAX_PWM

MIN_PWM = 80             # slowest PWM (0-255) that still moves the car; raise it if it stalls
MAX_PWM = 170            # fastest PWM; lower it if the car overshoots
MAX_STEP = 50            # max PWM change per control tick, so the car doesn't jerk

LOST_TIMEOUT = 0.5       # stop if no minifig has been seen for this many seconds
LOOP_PERIOD = 0.05       # control tick; the sketch stops itself after ~400 ms of silence

_lock = threading.Lock()
_latest = {"position": None, "received": 0.0}
_current = 0               # last commanded PWM, for ramping
_last_status = None


def axis_command(error: float) -> float:
    """Signed PWM (-MAX_PWM..MAX_PWM) with the sign of `error`; 0 inside the deadband."""
    magnitude = abs(error)
    if magnitude <= DEADBAND:
        return 0.0
    # Ramp from MIN_PWM just outside the deadband to MAX_PWM at FULL_SPEED_ERROR, so the
    # car slows down as it approaches the centre.
    fraction = min(1.0, (magnitude - DEADBAND) / (FULL_SPEED_ERROR - DEADBAND))
    pwm = MIN_PWM + fraction * (MAX_PWM - MIN_PWM)
    return pwm if error > 0 else -pwm


def compute_speed(position: float) -> int:
    """Signed PWM (-MAX_PWM..MAX_PWM) for both motors that moves the minifig towards TARGET."""
    error = position - TARGET
    # The minifig is past the centre in the + direction, so move it in the - direction.
    speed = -axis_command(error)
    return round(speed if FORWARD_INCREASES else -speed)


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

    if position is None or time.monotonic() - received > LOST_TIMEOUT:
        target, status, position = 0, "no minifig seen - stopped", None
    else:
        target = compute_speed(position)
        if target == 0:
            status = "minifig centred - stopped"
        else:
            status = "driving forward" if target > 0 else "driving backward"

    speed = ramp(_current, target)

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
