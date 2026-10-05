"""Green Minifig Finder car - Linux side of the Arduino UNO Q App Lab app.

Subscribes to the minifig positions published by minifig_mqtt.py and drives the car
forward/backward until the minifig is centred in the camera frame. Motor speeds are sent
to the sketch (sketch/sketch.ino) over the Bridge.
"""

import json
import threading
import time

import paho.mqtt.client as mqtt
from arduino.app_utils import App, Bridge

# ---- MQTT -------------------------------------------------------------------------
MQTT_BROKER = "10.243.102.145"  # IP address of the Mac running Mosquitto
MQTT_PORT = 1883
MQTT_USERNAME = ""               # leave blank: the server accepts anyone
MQTT_PASSWORD = ""
MQTT_TOPIC = "minifig/detections"

# ---- Steering ---------------------------------------------------------------------
# Which image axis the car moves along: "x" (left/right in the frame) or "y" (up/down).
AXIS = "x"
# True if driving forward moves the minifig towards higher x/y (right/down in the frame).
# If the car drives away from the centre instead of towards it, flip this.
FORWARD_INCREASES = True

TARGET = 0.5             # centre of the frame
DEADBAND = 0.04          # "centred" when within 4% of the frame from TARGET
FULL_SPEED_ERROR = 0.25  # at this distance from TARGET or further, drive at MAX_PWM
MIN_PWM = 90             # slowest PWM (0-255) that still moves the car; raise it if the car stalls
MAX_PWM = 180            # fastest PWM; lower it if the car overshoots the centre
LOST_TIMEOUT = 0.5       # stop if no minifig has been seen for this many seconds
LOOP_PERIOD = 0.05       # send a motor command every 50 ms (the sketch stops after 500 ms of silence)

_lock = threading.Lock()
_latest = {"position": None, "received": 0.0}
_last_status = None


def compute_speed(position: float) -> int:
    """Signed motor PWM (-255..255) that moves the minifig towards TARGET."""
    error = position - TARGET
    if abs(error) <= DEADBAND:
        return 0

    # Ramp from MIN_PWM just outside the deadband up to MAX_PWM at FULL_SPEED_ERROR,
    # so the car slows down as it approaches the centre.
    fraction = min(1.0, (abs(error) - DEADBAND) / (FULL_SPEED_ERROR - DEADBAND))
    pwm = round(MIN_PWM + fraction * (MAX_PWM - MIN_PWM))

    # The minifig is past the centre in the + direction, so move it in the - direction.
    direction = -1 if error > 0 else 1
    if not FORWARD_INCREASES:
        direction = -direction
    return direction * pwm


def on_connect(client, userdata, flags, reason_code, properties=None):
    if getattr(reason_code, "is_failure", reason_code != 0):
        print(f"MQTT connection refused: {reason_code}")
        return
    print(f"Connected to MQTT broker {MQTT_BROKER}:{MQTT_PORT}, listening on '{MQTT_TOPIC}'")
    client.subscribe(MQTT_TOPIC)


def on_message(client, userdata, msg):
    try:
        data = json.loads(msg.payload)
        detections = data.get("detections") or []
        position = None
        if detections:
            best = max(detections, key=lambda d: d["confidence"])
            position = best["center_norm"][AXIS]
    except (ValueError, KeyError, TypeError) as e:
        print(f"Ignoring bad message: {e}")
        return

    with _lock:
        _latest["position"] = position
        _latest["received"] = time.monotonic()


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
    client.connect_async(MQTT_BROKER, MQTT_PORT, keepalive=30)  # keeps retrying until the Mac is up
    client.loop_start()
    return client


def loop():
    global _last_status

    with _lock:
        position, received = _latest["position"], _latest["received"]

    if position is None or time.monotonic() - received > LOST_TIMEOUT:
        position, speed, status = None, 0, "no minifig seen - stopped"
    else:
        speed = compute_speed(position)
        if speed == 0:
            status = "minifig centred - stopped"
        else:
            status = "driving forward" if speed > 0 else "driving backward"

    # Sent every loop, even when stopped, so the sketch's watchdog knows we're alive.
    Bridge.call("set_motors", speed, speed)

    if status != _last_status:
        where = "" if position is None else f" (minifig at {AXIS}={position:.2f})"
        print(f"{status}{where}")
        _last_status = status

    time.sleep(LOOP_PERIOD)


start_mqtt()
App.run(user_loop=loop)
