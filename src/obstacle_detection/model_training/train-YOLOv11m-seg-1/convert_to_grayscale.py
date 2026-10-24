from pathlib import Path

from PIL import Image


DATASET_DIR = Path(__file__).parent.parent / "YOLOv11m-seg-1"
SPLITS = ("train", "valid", "test")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def main():
    converted = 0

    for split in SPLITS:
        split_dir = DATASET_DIR / split
        # YOLO exports put images in images/; COCO exports commonly put them
        # directly in the split folder. Prefer images/ when it exists.
        image_dir = split_dir / "images" if (split_dir / "images").is_dir() else split_dir
        if not image_dir.is_dir():
            print(f"Skipping missing folder: {image_dir}")
            continue

        for path in image_dir.iterdir():
            if not path.is_file() or path.suffix.lower() not in IMAGE_EXTENSIONS:
                continue

            # Keep three channels for compatibility with the default YOLO model.
            with Image.open(path) as image:
                grayscale_rgb = image.convert("L").convert("RGB")
                grayscale_rgb.save(path)
            converted += 1

    print(f"Converted {converted} images to grayscale.")


if __name__ == "__main__":
    main()
