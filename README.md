# Green Minifig Finder

Detects green LEGO minifigures with a YOLOv8 model and publishes their locations as JSON over MQTT.

## Setup

```bash
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt
```

## Detect minifigs and publish over MQTT

The trained model is included at `runs/green_minifig/weights/best.pt`.

```bash
.venv/bin/python minifig_mqtt.py --broker 192.168.1.50          # webcam -> MQTT server
.venv/bin/python minifig_mqtt.py --source photo.jpg --dry-run    # print the JSON instead of sending it
.venv/bin/python minifig_mqtt.py --only-detections --topic robot/minifig --qos 1
```

Each frame is published to `minifig/detections` (by default) as:

```json
{
  "timestamp": "2026-10-05T18:30:00.123456+00:00",
  "source": "IMG_7278.jpg",
  "frame": 0,
  "image": {"width": 512, "height": 512},
  "count": 1,
  "detections": [{
    "id": 0,
    "label": "green-lego-minifigure-detector",
    "confidence": 0.957,
    "center": {"x": 493.5, "y": 77.9},
    "center_norm": {"x": 0.9638, "y": 0.1521},
    "bbox": {"x1": 475.0, "y1": 47.7, "x2": 512.0, "y2": 108.0},
    "size": {"width": 37.0, "height": 60.3}
  }]
}
```

`center` is in pixels; `center_norm` is the same point as a fraction of the image (0–1, top-left is 0,0).

Run `python minifig_mqtt.py --help` for all options (username/password, confidence threshold, custom weights).

## Retrain the model

The dataset (`Green LEGO Minifigure Detector.v1-green-minifig-v0.yolov8/`) is a Roboflow YOLOv8 export: 54 train, 12 validation and 7 test images, one class.

```bash
.venv/bin/python train_yolov8.py                                  # YOLOv8n, 100 epochs
.venv/bin/python train_yolov8.py --model yolov8s.pt --epochs 150   # larger model
.venv/bin/python train_yolov8.py --export onnx                     # also export to ONNX/CoreML/TFLite
.venv/bin/python train_yolov8.py --predict path/to/photo.jpg       # save annotated predictions
```

Current model (YOLOv8n) on the test split: precision 0.954, recall 1.0, mAP50 0.995, mAP50-95 0.885. The test set is only 7 images, so expect lower accuracy on new photos.
