# Green Minifig Finder

Detects green LEGO minifigures with a YOLOv8 model, publishes their positions as JSON over MQTT, and drives an Arduino UNO Q car until the minifig is centred in the camera frame.

```
Mac webcam -> minifig_mqtt.py (YOLOv8) -> Mosquitto MQTT server -> UNO Q (unoq_car/) -> MakerDrive -> motors
```

## One-time setup (Mac)

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
brew install mosquitto
```

For the car, also set up the UNO Q app once: see [unoq_car/README.md](unoq_car/README.md).

## Running it

Run each step in its own terminal, from the project folder.

**1. Start the MQTT server** (leave it running):

```bash
/opt/homebrew/opt/mosquitto/sbin/mosquitto -c mosquitto.conf
```

The server listens on port 1883 and accepts any device that can reach this Mac, with no password. Anyone on the same network can read the positions or publish fake ones, which would move the car.

**2. Start the detector** (leave it running):

```bash
.venv/bin/python minifig_mqtt.py
```

A preview window shows the webcam with a box around each minifig. Quit at any time with `q` or `Esc` (in the window or the terminal), by closing the window, or with Ctrl-C.

**3. (Optional) Watch the messages:**

```bash
mosquitto_sub -h localhost -t minifig/detections
```

**4. Start the car** over SSH on the UNO Q ([unoq_car/README.md](unoq_car/README.md)):

```bash
ssh arduino@<UNOQ_IP>
arduino-app-cli app start ~/ArduinoApps/unoq_car
arduino-app-cli app logs ~/ArduinoApps/unoq_car     # watch what the car is doing
arduino-app-cli app stop ~/ArduinoApps/unoq_car     # stop the car
```

## Detector options

```bash
.venv/bin/python minifig_mqtt.py --dry-run                       # print the JSON instead of sending it (no server needed)
.venv/bin/python minifig_mqtt.py --camera 1                      # pick a webcam (default: first one that works)
.venv/bin/python minifig_mqtt.py --no-show                       # no preview window
.venv/bin/python minifig_mqtt.py --broker 192.168.1.50           # send to an MQTT server on another computer
.venv/bin/python minifig_mqtt.py --only-detections --topic robot/minifig --qos 1
.venv/bin/python minifig_mqtt.py --source photo.jpg              # image, folder, video or stream URL instead of the webcam
```

By default every frame is sent; use `--interval 0.1` to send at most one message every 0.1 s. Run `.venv/bin/python minifig_mqtt.py --help` for all options.

Each message is published to `minifig/detections` (by default) as:

```json
{
  "timestamp": "2026-10-05T18:30:00.123456+00:00",
  "source": "webcam:1",
  "frame": 0,
  "image": {"width": 1920, "height": 1080},
  "count": 1,
  "detections": [{
    "id": 0,
    "label": "green-lego-minifigure-detector",
    "confidence": 0.957,
    "center": {"x": 912.4, "y": 480.1},
    "center_norm": {"x": 0.4752, "y": 0.4445},
    "bbox": {"x1": 870.0, "y1": 400.5, "x2": 954.8, "y2": 559.7},
    "size": {"width": 84.8, "height": 159.2}
  }]
}
```

`center` is in pixels; `center_norm` is the same point as a fraction of the image (0–1, top-left is 0,0).

## Retrain the model

The dataset (`Green LEGO Minifigure Detector.v1-green-minifig-v0.yolov8/`) is a Roboflow YOLOv8 export: 54 train, 12 validation and 7 test images, one class. The trained model is at `runs/green_minifig/weights/best.pt`.

```bash
.venv/bin/python train_yolov8.py                                  # YOLOv8n, 100 epochs
.venv/bin/python train_yolov8.py --model yolov8s.pt --epochs 150   # larger model
.venv/bin/python train_yolov8.py --export onnx                     # also export to ONNX/CoreML/TFLite
.venv/bin/python train_yolov8.py --predict path/to/photo.jpg       # save annotated predictions
```

Current model (YOLOv8n) on the test split: precision 0.954, recall 1.0, mAP50 0.995, mAP50-95 0.885. The test set is only 7 images, so expect lower accuracy on new photos.
