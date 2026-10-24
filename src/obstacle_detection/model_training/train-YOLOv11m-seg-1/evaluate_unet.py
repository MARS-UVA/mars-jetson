"""Evaluate a saved U-Net++ checkpoint and save predicted binary masks."""

import argparse
from pathlib import Path

import segmentation_models_pytorch as smp
import torch
from PIL import Image
from torch.utils.data import DataLoader

from train_unet import TraversabilityDataset


def main():
    script_dir = Path(__file__).parent
    repo_dir = script_dir.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint",
        type=Path,
        default=script_dir / "unetplusplus_best.pth",
    )
    parser.add_argument(
        "--data-dir",
        type=Path,
        default=repo_dir / "YOLOv11m-seg-1-cleaned",
    )
    parser.add_argument("--split", choices=("train", "valid", "test"), default="test")
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=script_dir / "predictions",
        help="Directory for the predicted 0/255 PNG masks.",
    )
    args = parser.parse_args()

    if not args.checkpoint.is_file():
        parser.error(f"Checkpoint not found: {args.checkpoint}")

    device = torch.device(
        "cuda" if torch.cuda.is_available()
        else "mps" if torch.backends.mps.is_available()
        else "cpu"
    )
    checkpoint = torch.load(args.checkpoint, map_location=device, weights_only=False)
    image_size = (
        checkpoint.get("image_width", checkpoint.get("image_size", 512)),
        checkpoint.get("image_height", checkpoint.get("image_size", 512)),
    )

    dataset = TraversabilityDataset(
        args.data_dir / args.split,
        image_size=image_size,
        augment=False,
    )
    loader = DataLoader(
        dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )

    model = smp.UnetPlusPlus(
        encoder_name="resnet34",
        encoder_weights=None,
        in_channels=checkpoint.get("in_channels", 1),
        classes=1,
        activation=None,
    ).to(device)
    model.load_state_dict(checkpoint["model_state_dict"])
    model.eval()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    intersection = predicted_pixels = actual_pixels = union = 0
    sample_index = 0

    with torch.no_grad():
        for images, masks in loader:
            images = images.to(device, non_blocking=True)
            masks = masks.to(device, non_blocking=True)
            predictions = torch.sigmoid(model(images)) >= 0.5
            targets = masks >= 0.5

            intersection += (predictions & targets).sum().item()
            predicted_pixels += predictions.sum().item()
            actual_pixels += targets.sum().item()
            union += (predictions | targets).sum().item()

            for prediction in predictions.cpu():
                image_path, _ = dataset.samples[sample_index]
                output = prediction.squeeze(0).numpy().astype("uint8") * 255
                Image.fromarray(output, mode="L").save(
                    args.output_dir / f"{image_path.stem}.png"
                )
                sample_index += 1

    dice = (2 * intersection + 1e-7) / (predicted_pixels + actual_pixels + 1e-7)
    iou = (intersection + 1e-7) / (union + 1e-7)
    print(f"Checkpoint: {args.checkpoint}")
    print(f"Split: {args.split} ({len(dataset)} images)")
    print(f"Dice: {dice:.4f} | IoU: {iou:.4f}")
    print(f"Predicted masks saved to: {args.output_dir}")


if __name__ == "__main__":
    main()
