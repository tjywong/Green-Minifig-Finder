# Minifig Car (Arduino UNO Q)

Receives the green minifig's position as JSON over MQTT (published by `minifig_mqtt.py` on topic `minifig/detections`) and drives a two-motor car until the minifig is in the centre of the camera frame.

## Wiring (Cytron MakerDrive)

| UNO Q | MakerDrive | Motor |
|-------|-----------|-------|
| D5 (PWM)  | M1A | left  |
| D6 (PWM)  | M1B | left  |
| D9 (PWM)  | M2A | right |
| D10 (PWM) | M2B | right |
| GND       | GND | |

Power the motors from the MakerDrive's own battery input and share GND with the UNO Q.

## How it works

- `python/main.py` (Linux side) subscribes to MQTT, takes the highest-confidence detection's `center_norm` along `AXIS`, and every 50 ms sends the **same** PWM to both motors, so the car only drives forward or backward (no steering):
  - far from the centre (more than `PULSE_ZONE`): drives continuously, ramping from `MIN_PWM` up to `MAX_PWM`
  - close to the centre: creeps in short pulses (`PULSE_ON` seconds), then waits `PULSE_SETTLE` and judges the position from `SETTLED_FRAMES` frames taken after the car stopped, so camera/network delay doesn't make it overshoot
  - within `STOP_BAND` of the centre: stops, and stays stopped until the minifig is more than `RESTART_BAND` off (so detection jitter doesn't make it twitch)
  - positions are the median of the last `SMOOTH_WINDOW` seconds of frames, so one jumpy detection can't stop the car
  - the power adapts to the car: a pulse that doesn't move the minifig makes the next one `PULSE_BOOST` stronger, and continuous driving gets boosted if the minifig hasn't moved for `STALL_TIME`. Each pulse decision is printed in the log (`settled at x=... -> pulse forward at PWM ...`)
  - no detection for 0.5 s → stop
- `sketch/sketch.ino` (MCU side) exposes `set_motors(left, right)` over the Bridge, drives the MakerDrive, and stops the motors if commands stop arriving for 400 ms.

## Tuning

Edit the constants at the top of `python/main.py`:

- `MQTT_BROKER`: IP of the computer running Mosquitto.
- `AXIS` (`"x"` or `"y"`): which image axis the car moves along. If the car drives away from the centre, flip `FORWARD_INCREASES`.
- `MIN_PWM` (raise if the car stalls), `MAX_PWM` (lower if it overshoots into the pulse zone).
- `PULSE_PWM` (starting pulse power; it is boosted automatically if the car doesn't move), `PULSE_ON` (lower for smaller steps), `PULSE_SETTLE` (raise if it still overshoots near the centre).
- `STOP_BAND` / `RESTART_BAND`: how close counts as centred. Keep `minifig_mqtt.py --deadband` equal to `STOP_BAND` so the preview's green zone matches.
- In `sketch/sketch.ino`, flip `MOTOR1_REVERSED` / `MOTOR2_REVERSED` if a wheel spins backward on a forward command.

## Run

```bash
arduino-app-cli app start ~/ArduinoApps/unoq_car
arduino-app-cli app logs  ~/ArduinoApps/unoq_car --follow
```

Test with the wheels off the ground first.
