"""Compare grayscale images with predicted masks and red overlays."""

import argparse
import random
from pathlib import Path

from PIL import Image, ImageDraw


IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}
PANEL_SIZE = (384, 288)
HEADER_HEIGHT = 34
PANEL_GAP = 12
ROW_GAP = 18


def make_panel(image, size, resample):
    """Fit an image inside a fixed panel without changing its aspect ratio."""
    image = image.copy()
    image.thumbnail(size, resample)
    panel = Image.new("RGB", size, "#222222")
    offset = ((size[0] - image.width) // 2, (size[1] - image.height) // 2)
    panel.paste(image.convert("RGB"), offset)
    return panel


def make_overlay(image, mask):
    """Tint non-traversable mask pixels red over the grayscale image."""
    base = image.convert("RGB")
    mask = mask.resize(base.size, Image.Resampling.NEAREST)
    tint = Image.new("RGB", base.size, (255, 0, 0))
    alpha = mask.point(lambda value: 105 if value > 0 else 0)
    return Image.composite(tint, base, alpha)


def main():
    repo_dir = Path(__file__).parent.parent
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data-dir", type=Path, default=repo_dir / "YOLOv11m-seg-1")
    parser.add_argument("--split", choices=("train", "valid", "test"), default="test")
    parser.add_argument(
        "--predictions-dir",
        type=Path,
        default=Path(__file__).parent / "predictions",
        help="Folder containing predicted PNG masks named after their images.",
    )
    parser.add_argument("--count", type=int, default=6)
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path(__file__).parent / "mask_comparison.png",
    )
    args = parser.parse_args()

    if args.count < 1:
        parser.error("count must be at least 1")

    split_dir = args.data_dir / args.split
    nested_images = split_dir / "images"
    image_dir = nested_images if nested_images.is_dir() else split_dir
    if not args.predictions_dir.is_dir():
        parser.error(f"Predictions folder not found: {args.predictions_dir}")

    pairs = []
    for image_path in image_dir.iterdir():
        if not image_path.is_file() or image_path.suffix.lower() not in IMAGE_EXTENSIONS:
            continue
        prediction_path = args.predictions_dir / f"{image_path.stem}.png"
        if prediction_path.is_file():
            pairs.append((image_path, prediction_path))

    if not pairs:
        parser.error(
            f"No predictions matched images in {split_dir}. Run evaluate_unet.py "
            f"with --split {args.split} first, or provide --predictions-dir."
        )

    rng = random.Random(args.seed)
    selected = rng.sample(pairs, min(args.count, len(pairs)))

    columns = 3
    sheet_width = columns * PANEL_SIZE[0] + (columns + 1) * PANEL_GAP
    row_height = HEADER_HEIGHT + PANEL_SIZE[1] + ROW_GAP
    sheet_height = PANEL_GAP + len(selected) * row_height
    sheet = Image.new("RGB", (sheet_width, sheet_height), "#111111")
    draw = ImageDraw.Draw(sheet)
    headings = ("Grayscale image", "Predicted mask", "Overlay (predicted obstacle = red)")

    for row, (image_path, prediction_path) in enumerate(selected):
        with Image.open(image_path) as source:
            image = source.convert("L")
        with Image.open(prediction_path) as source_prediction:
            prediction = source_prediction.convert("L")

        panels = (
            make_panel(image, PANEL_SIZE, Image.Resampling.BILINEAR),
            make_panel(prediction, PANEL_SIZE, Image.Resampling.NEAREST),
            make_panel(
                make_overlay(image, prediction),
                PANEL_SIZE,
                Image.Resampling.BILINEAR,
            ),
        )
        y = PANEL_GAP + row * row_height
        draw.text((PANEL_GAP, y), image_path.name, fill="white")
        panel_y = y + HEADER_HEIGHT
        for column, (heading, panel) in enumerate(zip(headings, panels)):
            x = PANEL_GAP + column * (PANEL_SIZE[0] + PANEL_GAP)
            draw.text((x, panel_y), heading, fill="white")
            sheet.paste(panel, (x, panel_y + 18))

    args.output.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(args.output)
    print(f"Compared {len(selected)} pairs from the {args.split} split.")
    print(f"Saved comparison sheet to: {args.output}")


if __name__ == "__main__":
    main()
