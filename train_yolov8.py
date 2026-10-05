"""Train a YOLOv8 detector on the Green LEGO Minifigure dataset (Roboflow export).

Usage:
    python train_yolov8.py                 # train with defaults
    python train_yolov8.py --epochs 150 --model yolov8s.pt
    python train_yolov8.py --predict path/to/image.jpg   # run the trained model
"""

import argparse
from pathlib import Path

import torch
import yaml
from ultralytics import YOLO

ROOT = Path(__file__).resolve().parent
DATASET_DIR = ROOT / "Green LEGO Minifigure Detector.v1-green-minifig-v0.yolov8"
RUNS_DIR = ROOT / "runs"
RUN_NAME = "green_minifig"


def pick_device() -> str:
    if torch.cuda.is_available():
        return "0"
    if torch.backends.mps.is_available():
        return "mps"
    return "cpu"


def write_fixed_data_yaml() -> Path:
    """Roboflow's data.yaml uses '../train/images' paths that don't resolve from the
    dataset folder, so write a copy with absolute paths."""
    with open(DATASET_DIR / "data.yaml") as f:
        cfg = yaml.safe_load(f)

    fixed = {
        "path": str(DATASET_DIR),
        "train": "train/images",
        "val": "valid/images",
        "test": "test/images",
        "nc": cfg["nc"],
        "names": cfg["names"],
    }
    out = DATASET_DIR / "data_fixed.yaml"
    with open(out, "w") as f:
        yaml.safe_dump(fixed, f, sort_keys=False)
    return out


def train(args) -> Path:
    data_yaml = write_fixed_data_yaml()
    device = pick_device()
    print(f"Training {args.model} on {data_yaml} (device={device})")

    model = YOLO(args.model)  # pretrained COCO weights; downloaded automatically
    model.train(
        data=str(data_yaml),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=device,
        patience=30,  # early stop if val mAP stalls
        project=str(RUNS_DIR),
        name=RUN_NAME,
        exist_ok=True,
    )

    best = RUNS_DIR / RUN_NAME / "weights" / "best.pt"
    best_model = YOLO(str(best))

    print("\nEvaluating best weights on the test split:")
    metrics = best_model.val(data=str(data_yaml), split="test", device=device)
    print(f"test mAP50={metrics.box.map50:.3f}  mAP50-95={metrics.box.map:.3f}")

    if args.export:
        best_model.export(format=args.export)

    print(f"\nTrained model saved to: {best}")
    return best


def predict(args):
    weights = RUNS_DIR / RUN_NAME / "weights" / "best.pt"
    model = YOLO(str(weights))
    results = model.predict(args.predict, conf=args.conf, save=True, device=pick_device())
    for r in results:
        print(f"{r.path}: {len(r.boxes)} green minifig(s) detected")
    print(f"Annotated images saved to: {results[0].save_dir}")


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--model", default="yolov8n.pt", help="yolov8n/s/m/l/x.pt")
    p.add_argument("--epochs", type=int, default=100)
    p.add_argument("--imgsz", type=int, default=640)
    p.add_argument("--batch", type=int, default=16)
    p.add_argument("--export", default=None, help="optional export format, e.g. onnx, coreml, tflite")
    p.add_argument("--predict", default=None, help="image/folder/video to run the trained model on")
    p.add_argument("--conf", type=float, default=0.25)
    args = p.parse_args()

    if args.predict:
        predict(args)
    else:
        train(args)


if __name__ == "__main__":
    main()
