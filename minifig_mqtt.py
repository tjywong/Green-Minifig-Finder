"""Detect green LEGO minifigs and publish their locations over MQTT as JSON.

Usage:
    python minifig_mqtt.py                                  # first working webcam -> localhost:1883
    python minifig_mqtt.py --camera 1 --broker 192.168.1.50 # pick a specific webcam
    python minifig_mqtt.py --dry-run                        # webcam, print JSON instead of publishing
    python minifig_mqtt.py --source photo.jpg               # image, folder, video or stream URL

Webcam mode shows a live preview window with the detections (pass --no-show to run
headless). Quit at any time by pressing q or Esc in the preview window or the terminal,
closing the window, or pressing Ctrl-C.

Each published frame sends one message to --topic, e.g.:
{
  "timestamp": "2026-10-05T14:30:00.123456+00:00",
  "source": "webcam:1",
  "frame": 0,
  "image": {"width": 1920, "height": 1080},
  "count": 1,
  "detections": [
    {
      "id": 0,
      "label": "green-lego-minifigure-detector",
      "confidence": 0.91,
      "center": {"x": 912.4, "y": 480.1},
      "center_norm": {"x": 0.4752, "y": 0.4445},
      "bbox": {"x1": 870.0, "y1": 400.5, "x2": 954.8, "y2": 559.7},
      "size": {"width": 84.8, "height": 159.2}
    }
  ]
}
"""

import argparse
import json
import select
import signal
import sys
import termios
import threading
import time
import tty
from datetime import datetime, timezone
from pathlib import Path

import cv2
import paho.mqtt.client as mqtt
import torch
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DEFAULT_WEIGHTS = ROOT / "runs" / "green_minifig" / "weights" / "best.pt"
WINDOW_NAME = "Green Minifig Finder (q to quit)"
QUIT_KEYS = {ord("q"), ord("Q"), 27}  # 27 = Esc


def pick_device() -> str:
    if torch.cuda.is_available():
        return "0"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def result_to_message(result, frame_idx: int, source: str) -> dict:
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
        "source": source,
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


def open_camera(index, width, height):
    """Open the requested webcam, or the first one that actually returns frames.

    Some indexes (e.g. iPhone Continuity Camera or virtual cameras) open but never
    deliver an image, so each candidate is checked with a test read.
    """
    candidates = [index] if index is not None else range(5)
    for i in candidates:
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            if width:
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            if height:
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            ok, _ = cap.read()
            if ok:
                return cap, i
        cap.release()
    raise RuntimeError(
        f"No working webcam found (tried {list(candidates)}). On macOS, allow camera access for "
        "your terminal/VS Code in System Settings > Privacy & Security > Camera."
    )


def draw_center_line(frame, message, deadband):
    """Vertical line at the middle of the frame plus the stop zone (+/- deadband).

    Turns green when the most confident minifig's centroid is inside the zone, i.e. when
    the car (DEADBAND in unoq_car/python/main.py) considers it centred and stops.
    """
    height, width = frame.shape[:2]
    mid = width // 2
    band = int(deadband * width)

    best = max(message["detections"], key=lambda d: d["confidence"], default=None)
    error = None if best is None else best["center_norm"]["x"] - 0.5
    centred = error is not None and abs(error) <= deadband
    color = (0, 255, 0) if centred else (0, 255, 255)  # BGR: green when centred, else yellow

    cv2.line(frame, (mid - band, 0), (mid - band, height), color, 1)
    cv2.line(frame, (mid + band, 0), (mid + band, height), color, 1)
    cv2.line(frame, (mid, 0), (mid, height), color, 2)

    if error is not None:
        label = "CENTRED" if centred else f"off centre: {error:+.2f}"
        cv2.putText(frame, label, (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)


def draw_detections(frame, message, deadband):
    draw_center_line(frame, message, deadband)
    for det in message["detections"]:
        b, c = det["bbox"], det["center"]
        cv2.rectangle(frame, (int(b["x1"]), int(b["y1"])), (int(b["x2"]), int(b["y2"])), (0, 255, 0), 2)
        cv2.circle(frame, (int(c["x"]), int(c["y"])), 5, (0, 0, 255), -1)
        text = f"{det['confidence']:.2f}  ({det['center_norm']['x']:.2f}, {det['center_norm']['y']:.2f})"
        cv2.putText(frame, text, (int(b["x1"]), max(int(b["y1"]) - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
    cv2.putText(frame, f"minifigs: {message['count']}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)
    return frame


class Publisher:
    """Sends messages to MQTT (or prints them in --dry-run), at most once per --interval."""

    def __init__(self, args):
        self.args = args
        self.client = None if args.dry_run else connect(args)
        self.last_sent = 0.0

    def send(self, message: dict):
        if self.args.only_detections and message["count"] == 0:
            return
        now = time.monotonic()
        if now - self.last_sent < self.args.interval:
            return
        self.last_sent = now

        if self.client is None:
            print(json.dumps(message))
        else:
            info = self.client.publish(self.args.topic, json.dumps(message), qos=self.args.qos)
            info.wait_for_publish(timeout=2)
            centers = [(d["center_norm"]["x"], d["center_norm"]["y"]) for d in message["detections"]]
            print(f"[{self.args.topic}] frame {message['frame']}: {message['count']} minifig(s) {centers}")

    def close(self):
        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()


class TerminalQuitListener:
    """Sets `stop` when q (or Esc) is pressed in the terminal, no Enter needed.

    The preview window only sees keys while it has focus, so this lets you quit from
    the terminal too (and in --no-show mode).
    """

    def __init__(self):
        self.stop = threading.Event()
        self._saved_tty = None

    def __enter__(self):
        if sys.stdin.isatty():
            self._saved_tty = termios.tcgetattr(sys.stdin)
            tty.setcbreak(sys.stdin)  # read single keypresses; Ctrl-C still works
            threading.Thread(target=self._listen, daemon=True).start()
        return self

    def _listen(self):
        while not self.stop.is_set():
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready and ord(sys.stdin.read(1)) in QUIT_KEYS:
                self.stop.set()

    def __exit__(self, *exc):
        self.stop.set()
        if self._saved_tty is not None:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self._saved_tty)


def window_closed() -> bool:
    try:
        return cv2.getWindowProperty(WINDOW_NAME, cv2.WND_PROP_VISIBLE) < 1
    except cv2.error:
        return True


def run_webcam(model, publisher, args, stop):
    cap, index = open_camera(args.camera, args.width, args.height)
    source = f"webcam:{index}"
    print(f"Using {source} ({int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))})")
    print("Press q (in the preview window or this terminal) to quit.")
    device = pick_device()
    frame_idx = 0
    try:
        while not stop.is_set():
            ok, frame = cap.read()
            if not ok:
                print("Webcam stopped returning frames.")
                break

            result = model.predict(frame, conf=args.conf, device=device, verbose=False)[0]
            message = result_to_message(result, frame_idx, source)
            publisher.send(message)

            if args.show:
                cv2.imshow(WINDOW_NAME, draw_detections(frame, message, args.deadband))
                if (cv2.waitKey(1) & 0xFF) in QUIT_KEYS or window_closed():
                    break
            frame_idx += 1
    finally:
        cap.release()
        cv2.destroyAllWindows()
        cv2.waitKey(1)  # macOS needs an event-loop tick to actually close the window
        print("Stopped.")


def run_source(model, publisher, args, stop):
    # stream=True yields results one at a time instead of loading everything into memory
    results = model.predict(args.source, conf=args.conf, stream=True, device=pick_device(), verbose=False)
    for frame_idx, result in enumerate(results):
        if stop.is_set():
            break
        publisher.send(result_to_message(result, frame_idx, Path(result.path).name))


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--source", default=None, help="image, folder, video or stream URL (default: webcam)")
    p.add_argument("--camera", type=int, default=None, help="webcam index (default: first working one)")
    p.add_argument("--width", type=int, default=None, help="requested webcam width, e.g. 1280")
    p.add_argument("--height", type=int, default=None, help="requested webcam height, e.g. 720")
    p.add_argument("--no-show", dest="show", action="store_false", help="don't open a preview window")
    p.add_argument("--deadband", type=float, default=0.05,
                   help="half width of the centred zone drawn in the preview (match DEADBAND on the UNO Q)")
    p.add_argument("--weights", default=str(DEFAULT_WEIGHTS))
    p.add_argument("--conf", type=float, default=0.5, help="minimum detection confidence")
    p.add_argument("--interval", type=float, default=0.2, help="minimum seconds between messages (0 = every frame)")
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

    # treat `kill` like Ctrl-C so the camera, window and MQTT connection are cleaned up
    signal.signal(signal.SIGTERM, signal.default_int_handler)

    model = YOLO(args.weights)
    publisher = Publisher(args)
    try:
        with TerminalQuitListener() as listener:
            if args.source is None:
                run_webcam(model, publisher, args, listener.stop)
            elif args.source.isdigit():  # allow the old "--source 0" webcam syntax
                args.camera = int(args.source)
                run_webcam(model, publisher, args, listener.stop)
            else:
                args.interval = 0  # files: publish every image, no rate limit
                run_source(model, publisher, args, listener.stop)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.close()


if __name__ == "__main__":
    main()
