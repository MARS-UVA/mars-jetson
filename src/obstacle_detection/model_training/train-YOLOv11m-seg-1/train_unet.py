"""Train a binary U-Net++ model from split image and mask folders.

Expected layout (images may also be directly inside each split folder)::

    YOLOv11m-seg-1/
      train/images/  train/masks/
      valid/images/  valid/masks/
      test/images/   test/masks/

Masks use 0 for traversable and any nonzero value for non-traversable.
"""

import argparse
import random
from pathlib import Path

import segmentation_models_pytorch as smp
import torch
from PIL import Image, ImageOps
from torch import nn
from torch.utils.data import DataLoader, Dataset
from tqdm.auto import tqdm


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


class TraversabilityDataset(Dataset):
    """Load paired grayscale images and binary masks for one split."""

    def __init__(
        self, split_dir, image_size, augment=False, max_samples=None, sample_seed=0
    ):
        self.split_dir = Path(split_dir)
        nested_images = self.split_dir / "images"
        self.image_dir = nested_images if nested_images.is_dir() else self.split_dir
        self.mask_dir = self.split_dir / "masks"
        self.image_size = image_size
        self.augment = augment

        if not self.image_dir.is_dir():
            raise FileNotFoundError(f"Image folder not found: {self.image_dir}")
        if not self.mask_dir.is_dir():
            raise FileNotFoundError(f"Mask folder not found: {self.mask_dir}")

        self.samples = []
        missing_masks = []
        for image_path in sorted(self.image_dir.iterdir()):
            if not image_path.is_file() or image_path.suffix.lower() not in IMAGE_EXTENSIONS:
                continue
            mask_path = self.mask_dir / f"{image_path.stem}.png"
            if mask_path.is_file():
                self.samples.append((image_path, mask_path))
            else:
                missing_masks.append(mask_path)

        if missing_masks:
            examples = "\n".join(str(path) for path in missing_masks[:10])
            raise FileNotFoundError(
                f"{len(missing_masks)} image(s) have no matching PNG mask. Examples:\n{examples}"
            )
        if not self.samples:
            raise ValueError(f"No image/mask pairs found in {self.split_dir}")
        if max_samples is not None:
            if max_samples < 1:
                raise ValueError("max_samples must be positive")
            sample_rng = random.Random(sample_seed)
            self.samples = sample_rng.sample(
                self.samples, min(max_samples, len(self.samples))
            )

    def __len__(self):
        return len(self.samples)

    def __getitem__(self, index):
        image_path, mask_path = self.samples[index]
        with Image.open(image_path) as image_file:
            image = image_file.convert("L")
        with Image.open(mask_path) as mask_file:
            mask = mask_file.convert("L")

        image, mask = resize_pair(image, mask, self.image_size)

        if self.augment:
            if random.random() < 0.5:
                image = ImageOps.mirror(image)
                mask = ImageOps.mirror(mask)
            if random.random() < 0.5:
                image = ImageOps.flip(image)
                mask = ImageOps.flip(mask)

        image_tensor = image_to_tensor(image)
        mask_tensor = image_to_tensor(mask)
        image_tensor = image_tensor.float().div_(255.0)
        mask_tensor = (mask_tensor > 0).float()
        return image_tensor, mask_tensor


def image_to_tensor(image):
    """Convert a single-channel PIL image to a 1xHxW float tensor."""
    import numpy as np

    array = np.asarray(image, dtype="float32").copy()
    return torch.from_numpy(array).unsqueeze(0)


def resize_pair(image, mask, image_size):
    """Resize without changing aspect ratio, padding to (width, height)."""
    if isinstance(image_size, int):
        target_width = target_height = image_size
    else:
        target_width, target_height = image_size

    scale = min(target_width / image.width, target_height / image.height)
    resized_size = (
        max(1, round(image.width * scale)),
        max(1, round(image.height * scale)),
    )
    image = image.resize(resized_size, Image.Resampling.BILINEAR)
    mask = mask.resize(resized_size, Image.Resampling.NEAREST)

    canvas_size = (target_width, target_height)
    image_canvas = Image.new("L", canvas_size, 0)
    mask_canvas = Image.new("L", canvas_size, 0)
    offset = (
        (target_width - resized_size[0]) // 2,
        (target_height - resized_size[1]) // 2,
    )
    image_canvas.paste(image, offset)
    mask_canvas.paste(mask, offset)
    return image_canvas, mask_canvas


def dice_loss(logits, targets, smooth=1.0):
    probabilities = torch.sigmoid(logits)
    intersection = (probabilities * targets).sum(dim=(1, 2, 3))
    denominator = probabilities.sum(dim=(1, 2, 3)) + targets.sum(dim=(1, 2, 3))
    dice = (2.0 * intersection + smooth) / (denominator + smooth)
    return 1.0 - dice.mean()


def batch_scores(logits, targets):
    predictions = torch.sigmoid(logits) >= 0.5
    targets = targets >= 0.5
    intersection = (predictions & targets).sum().item()
    predicted = predictions.sum().item()
    actual = targets.sum().item()
    union = predicted + actual - intersection
    dice = (2 * intersection + 1e-7) / (predicted + actual + 1e-7)
    iou = (intersection + 1e-7) / (union + 1e-7)
    return intersection, predicted, actual, union, dice, iou


def run_epoch(model, loader, device, optimizer=None, description=""):
    training = optimizer is not None
    model.train(training)
    bce = nn.BCEWithLogitsLoss()
    total_loss = 0.0
    pixel_intersection = 0
    pixel_predicted = 0
    pixel_actual = 0
    pixel_union = 0

    context = torch.enable_grad if training else torch.no_grad
    with context():
        progress = tqdm(loader, desc=description, leave=False)
        for images, masks in progress:
            images = images.to(device, non_blocking=True)
            masks = masks.to(device, non_blocking=True)

            if training:
                optimizer.zero_grad(set_to_none=True)
            logits = model(images)
            loss = bce(logits, masks) + dice_loss(logits, masks)
            if training:
                loss.backward()
                optimizer.step()

            total_loss += loss.item() * images.size(0)
            progress.set_postfix(loss=f"{loss.item():.4f}")
            intersection, predicted, actual, union, _, _ = batch_scores(logits, masks)
            pixel_intersection += intersection
            pixel_predicted += predicted
            pixel_actual += actual
            pixel_union += union

    dice = (2 * pixel_intersection + 1e-7) / (
        pixel_predicted + pixel_actual + 1e-7
    )
    iou = (pixel_intersection + 1e-7) / (pixel_union + 1e-7)
    return total_loss / len(loader.dataset), dice, iou


def main():
    default_data_dir = Path(__file__).parent.parent / "YOLOv11m-seg-1"
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=default_data_dir)
    parser.add_argument("--epochs", type=int, default=50)
    parser.add_argument("--batch-size", type=int, default=4)
    parser.add_argument("--image-size", type=int, default=512)
    parser.add_argument(
        "--image-width",
        type=int,
        default=None,
        help="Input width; defaults to --image-size.",
    )
    parser.add_argument(
        "--image-height",
        type=int,
        default=None,
        help="Input height; defaults to --image-size.",
    )
    parser.add_argument("--learning-rate", type=float, default=1e-4)
    parser.add_argument("--workers", type=int, default=0)
    parser.add_argument(
        "--sample-seed",
        type=int,
        default=42,
        help="Seed used to randomly choose capped train/validation/test subsets.",
    )
    parser.add_argument(
        "--device",
        choices=("auto", "cpu", "cuda", "mps"),
        default="auto",
        help="Compute device; auto selects CUDA, then Apple Metal (MPS), then CPU.",
    )
    parser.add_argument(
        "--max-train-samples",
        type=int,
        default=None,
        help="Optional cap for a quick smoke run; uses the first N training pairs.",
    )
    parser.add_argument(
        "--max-valid-samples",
        type=int,
        default=None,
        help="Optional cap for validation pairs during a smoke run.",
    )
    parser.add_argument(
        "--max-test-samples",
        type=int,
        default=None,
        help="Optional cap for test pairs during a smoke run.",
    )
    parser.add_argument(
        "--encoder-weights",
        choices=("imagenet", "none"),
        default="imagenet",
        help="Use ImageNet pretrained ResNet34 weights, or initialize from scratch.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "unetplusplus_best.pth",
    )
    args = parser.parse_args()

    image_width = args.image_width or args.image_size
    image_height = args.image_height or args.image_size
    if args.epochs < 1 or args.batch_size < 1:
        parser.error("epochs and batch-size must be positive")
    if image_width < 32 or image_height < 32:
        parser.error("image width and height must each be at least 32")
    if image_width % 32 or image_height % 32:
        parser.error("image width and height must each be divisible by 32")

    if args.device == "auto":
        device_name = (
            "cuda" if torch.cuda.is_available()
            else "mps" if torch.backends.mps.is_available()
            else "cpu"
        )
    else:
        device_name = args.device
        if device_name == "cuda" and not torch.cuda.is_available():
            parser.error("CUDA was requested but is not available")
        if device_name == "mps" and not torch.backends.mps.is_available():
            parser.error("MPS/Metal was requested but is not available in this PyTorch environment")
    device = torch.device(device_name)
    print(f"Training on {device}")

    train_dataset = TraversabilityDataset(
        args.data_dir / "train", (image_width, image_height), augment=True,
        max_samples=args.max_train_samples,
        sample_seed=args.sample_seed,
    )
    valid_dataset = TraversabilityDataset(
        args.data_dir / "valid", (image_width, image_height),
        max_samples=args.max_valid_samples,
        sample_seed=args.sample_seed + 1,
    )
    test_dataset = TraversabilityDataset(
        args.data_dir / "test", (image_width, image_height),
        max_samples=args.max_test_samples,
        sample_seed=args.sample_seed + 2,
    )
    print(
        f"Using {len(train_dataset)} train, {len(valid_dataset)} validation, "
        f"and {len(test_dataset)} test image/mask pairs."
    )
    train_loader = DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )
    valid_loader = DataLoader(
        valid_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )
    test_loader = DataLoader(
        test_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.workers,
        pin_memory=device.type == "cuda",
    )

    encoder_weights = None if args.encoder_weights == "none" else "imagenet"
    model = smp.UnetPlusPlus(
        encoder_name="resnet34",
        encoder_weights=encoder_weights,
        in_channels=1,
        classes=1,
        activation=None,
    ).to(device)
    optimizer = torch.optim.AdamW(model.parameters(), lr=args.learning_rate)

    best_dice = -1.0
    for epoch in range(1, args.epochs + 1):
        train_loss, train_dice, train_iou = run_epoch(
            model,
            train_loader,
            device,
            optimizer,
            description=f"Train {epoch}/{args.epochs}",
        )
        valid_loss, valid_dice, valid_iou = run_epoch(
            model,
            valid_loader,
            device,
            description=f"Valid {epoch}/{args.epochs}",
        )
        print(
            f"Epoch {epoch:03d}/{args.epochs} "
            f"train loss={train_loss:.4f} dice={train_dice:.4f} IoU={train_iou:.4f} | "
            f"valid loss={valid_loss:.4f} dice={valid_dice:.4f} IoU={valid_iou:.4f}"
        )

        if valid_dice > best_dice:
            best_dice = valid_dice
            args.output.parent.mkdir(parents=True, exist_ok=True)
            torch.save(
                {
                    "model_state_dict": model.state_dict(),
                    "epoch": epoch,
                    "valid_dice": valid_dice,
                    "image_size": args.image_size,
                    "image_width": image_width,
                    "image_height": image_height,
                    "in_channels": 1,
                    "mask_values": {"traversable": 0, "non_traversable": 255},
                },
                args.output,
            )

    best_checkpoint = torch.load(args.output, map_location=device)
    model.load_state_dict(best_checkpoint["model_state_dict"])
    test_loss, test_dice, test_iou = run_epoch(
        model, test_loader, device, description="Test"
    )
    print(f"Best validation Dice: {best_dice:.4f}")
    print(f"Test loss={test_loss:.4f} Dice={test_dice:.4f} IoU={test_iou:.4f}")
    print(f"Saved best checkpoint to {args.output}")


if __name__ == "__main__":
    main()
