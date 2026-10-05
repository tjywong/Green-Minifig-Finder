# UNO Q minifig car

Arduino UNO Q App Lab app that listens for minifig positions over MQTT and drives a two-motor car forward or backward until the minifig is centred in the camera frame.

- `python/main.py` runs on the UNO Q's Linux side. It subscribes to `minifig/detections`, decides the speed and sends it to the sketch over the Bridge.
- `sketch/sketch.ino` runs on the UNO Q's microcontroller. It drives the Cytron MakerDrive and stops the motors if no command arrives for 500 ms.

## Wiring

| UNO Q | Cytron MakerDrive |
|---|---|
| D5 | M1A |
| D6 | M1B |
| D9 | M2A |
| D10 | M2B |
| GND | GND |

Power the motors from the MakerDrive's own supply input, not from the UNO Q.

## One-time setup

1. Connect the UNO Q to the same Wi-Fi network as the Mac and find its IP address. You can get it from App Lab, from your router, or by running `ip addr show wlan0` on the board.
2. In `python/main.py`, set `MQTT_BROKER` to the Mac's IP address (run `ipconfig getifaddr en0` on the Mac). No username or password is needed.
3. From the project folder on the Mac, copy the app to the board (replace `<UNOQ_IP>`):
   ```bash
   ssh arduino@<UNOQ_IP> "mkdir -p ~/ArduinoApps/unoq_car"
   scp -r unoq_car/* arduino@<UNOQ_IP>:~/ArduinoApps/unoq_car/
   ```
   Re-run the `scp` line whenever you change the code on the Mac.

## Running over SSH

1. On the Mac, start the MQTT server and the detector (see [the main README](../README.md#running-it)).
2. SSH into the UNO Q and start the app. This compiles and uploads the sketch, installs `paho-mqtt`, and starts `main.py`:
   ```bash
   ssh arduino@<UNOQ_IP>
   arduino-app-cli app start ~/ArduinoApps/unoq_car
   ```
3. Watch what the car is doing. The Python program runs in a container, so its output goes to the app logs rather than your SSH terminal:
   ```bash
   arduino-app-cli app logs ~/ArduinoApps/unoq_car
   ```
   You'll see lines like `Connected to MQTT broker ...`, `driving forward (minifig at x=0.20)` and `minifig centred - stopped`.
4. Stop the car:
   ```bash
   arduino-app-cli app stop ~/ArduinoApps/unoq_car
   ```
   The car also stops by itself if the minifig leaves the frame or the Mac stops sending positions.

The folder is also a normal App Lab app, so you can run it from App Lab instead if you prefer.

## Calibration

Do these steps with the car's wheels off the ground first.

1. **Wheel direction:** both wheels should spin the same way. If one spins backward, set `MOTOR1_REVERSED` or `MOTOR2_REVERSED` in `sketch.ino`.
2. **Driving direction:** place the car so the minifig is off-centre. If the car drives away from the centre, flip `FORWARD_INCREASES` in `main.py`.
3. **Axis:** if the car moves up/down in the camera image rather than left/right, set `AXIS = "y"`.
4. **Speed:**
   - If the car stalls near the centre, raise `MIN_PWM`.
   - If it overshoots and oscillates, lower `MAX_PWM` or widen `DEADBAND`.

## Troubleshooting

- **App never prints "Connected to MQTT broker":** the UNO Q can't reach the Mac.
  - Check that `MQTT_BROKER` matches the Mac's current IP; it can change when the Mac reconnects to Wi-Fi.
  - Check that the MQTT server is running.
  - Campus networks such as eduroam often block devices from talking to each other. If so, put both devices on a phone hotspot or your own router.
- **Connected but the car never moves:** run `mosquitto_sub -h localhost -t minifig/detections` on the Mac to check that positions are being sent and that `count` is above 0.
