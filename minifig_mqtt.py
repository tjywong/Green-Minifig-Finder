"""Detect green LEGO minifigs and publish their locations over MQTT as JSON.

Usage:
    python minifig_mqtt.py                                  # webcam 0 -> localhost:1883
    python minifig_mqtt.py --source photo.jpg --broker 192.168.1.50
    python minifig_mqtt.py --source images/ --dry-run       # print JSON, don't publish

Each processed frame publishes one message to --topic, e.g.:
{
  "timestamp": "2026-10-05T14:30:00.123456+00:00",
  "source": "photo.jpg",
  "frame": 0,
  "image": {"width": 640, "height": 640},
  "count": 1,
  "detections": [
    {
      "id": 0,
      "label": "green-lego-minifigure-detector",
      "confidence": 0.91,
      "center": {"x": 312.4, "y": 280.1},
      "center_norm": {"x": 0.488, "y": 0.438},
      "bbox": {"x1": 270.0, "y1": 200.5, "x2": 354.8, "y2": 359.7},
      "size": {"width": 84.8, "height": 159.2}
    }
  ]
}
"""

import argparse
import json
from datetime import datetime, timezone
from pathlib import Path

import paho.mqtt.client as mqtt
import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DEFAULT_WEIGHTS = ROOT / "runs" / "green_minifig" / "weights" / "best.pt"


def pick_device() -> str:
    if torch.cuda.is_available():
        return "0"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def result_to_message(result, frame_idx: int) -> dict:
    """Convert one Ultralytics result into a JSON-serialisable location message."""
    height, width = result.orig_shape
    detections = []
    for i, (xyxy, conf, cls) in enumerate(
        zip(result.boxes.xyxy.tolist(), result.boxes.conf.tolist(), result.boxes.cls.tolist())
    ):
        x1, y1, x2, y2 = xyxy
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        detections.append({
            "id": i,
            "label": result.names[int(cls)],
            "confidence": round(conf, 3),
            "center": {"x": round(cx, 1), "y": round(cy, 1)},
            "center_norm": {"x": round(cx / width, 4), "y": round(cy / height, 4)},
            "bbox": {"x1": round(x1, 1), "y1": round(y1, 1), "x2": round(x2, 1), "y2": round(y2, 1)},
            "size": {"width": round(x2 - x1, 1), "height": round(y2 - y1, 1)},
        })

    return {
        "timestamp": datetime.now(timezone.utc).isoformat(),
        "source": Path(result.path).name if result.path else None,
        "frame": frame_idx,
        "image": {"width": width, "height": height},
        "count": len(detections),
        "detections": detections,
    }


def connect(args) -> mqtt.Client:
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, client_id=args.client_id)
    if args.username:
        client.username_pw_set(args.username, args.password)
    client.connect(args.broker, args.port, keepalive=60)
    client.loop_start()  # background network thread handles acks/reconnects
    return client


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default="0", help="image, folder, video, stream URL, or webcam index")
    p.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    p.add_argument("--conf", type=float, default=0.5, help="minimum detection confidence")
    p.add_argument("--broker", default="localhost")
    p.add_argument("--port", type=int, default=1883)
    p.add_argument("--topic", default="minifig/detections")
    p.add_argument("--qos", type=int, choices=[0, 1, 2], default=0)
    p.add_argument("--client-id", default="green-minifig-detector")
    p.add_argument("--username", default=None)
    p.add_argument("--password", default=None)
    p.add_argument("--only-detections", action="store_true", help="skip frames with no minifigs")
    p.add_argument("--dry-run", action="store_true", help="print JSON instead of publishing")
    args = p.parse_args()

    source = int(args.source) if args.source.isdigit() else args.source
    model = YOLO(args.weights)
    client = None if args.dry_run else connect(args)

    try:
        # stream=True yields results one frame at a time (needed for webcam/video)
        results = model.predict(source, conf=args.conf, stream=True, device=pick_device(), verbose=False)
        for frame_idx, result in enumerate(results):
            message = result_to_message(result, frame_idx)
            if args.only_detections and message["count"] == 0:
                continue

            payload = json.dumps(message)
            if client is None:
                print(json.dumps(message, indent=2))
            else:
                client.publish(args.topic, payload, qos=args.qos).wait_for_publish()
                print(f"[{args.topic}] frame {frame_idx}: {message['count']} minifig(s)")
    except KeyboardInterrupt:
        pass
    finally:
        if client is not None:
            client.loop_stop()
            client.disconnect()


if __name__ == "__main__":
    main()
