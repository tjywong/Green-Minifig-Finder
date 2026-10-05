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
  - proportional speed between `MIN_PWM` and `MAX_PWM`, a deadband around the centre, ramped acceleration
  - no detection for 0.5 s → stop
- `sketch/sketch.ino` (MCU side) exposes `set_motors(left, right)` over the Bridge, drives the MakerDrive, and stops the motors if commands stop arriving for 400 ms.

## Tuning

Edit the constants at the top of `python/main.py`:

- `MQTT_BROKER`: IP of the computer running Mosquitto.
- `AXIS` (`"x"` or `"y"`): which image axis the car moves along. If the car drives away from the centre, flip `FORWARD_INCREASES`.
- `MIN_PWM` (raise if the car stalls), `MAX_PWM` (lower if it overshoots), `DEADBAND`.
- In `sketch/sketch.ino`, flip `MOTOR1_REVERSED` / `MOTOR2_REVERSED` if a wheel spins backward on a forward command.

## Run

```bash
arduino-app-cli app start ~/ArduinoApps/unoq_car
arduino-app-cli app logs  ~/ArduinoApps/unoq_car --follow
```

Test with the wheels off the ground first.
