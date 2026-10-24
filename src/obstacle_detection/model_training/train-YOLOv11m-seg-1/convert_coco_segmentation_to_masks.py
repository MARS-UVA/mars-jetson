"""Convert a Roboflow COCO Segmentation export to binary pixel masks.

All annotated object regions are marked non-traversable (255); other pixels
are marked traversable (0). COCO polygon annotations are used, so the mask
follows the annotated object outline rather than an oriented bounding box.
"""

import json
from pathlib import Path

from PIL import Image, ImageDraw


DATASET_DIR = Path(__file__).parent.parent / "YOLOv11m-seg-1"
SPLITS = ("train", "valid", "test")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def find_coco_annotations(split_dir):
    """Find the COCO annotation JSON in a dataset split."""
    preferred = split_dir / "_annotations.coco.json"
    if preferred.is_file():
        return preferred

    json_files = sorted(split_dir.glob("*.json"))
    if len(json_files) == 1:
        return json_files[0]
    if not json_files:
        return None
    raise FileNotFoundError(
        f"Found multiple JSON files in {split_dir}; expected "
        "_annotations.coco.json."
    )


def find_image(split_dir, file_name):
    """Resolve a COCO file_name against common Roboflow export layouts."""
    relative_path = Path(file_name)
    candidates = (
        split_dir / relative_path,
        split_dir / "images" / relative_path,
        split_dir / relative_path.name,
        split_dir / "images" / relative_path.name,
    )
    for candidate in candidates:
        if candidate.is_file() and candidate.suffix.lower() in IMAGE_EXTENSIONS:
            return candidate

    # Some exports include a path prefix that differs from the local layout.
    for image_dir in (split_dir, split_dir / "images"):
        candidate = image_dir / relative_path.name
        if candidate.is_file() and candidate.suffix.lower() in IMAGE_EXTENSIONS:
            return candidate
    return None


def draw_coco_polygons(mask_draw, segmentation, width, height, annotation_id):
    """Rasterize polygon segmentation coordinates onto the mask."""
    if isinstance(segmentation, dict):
        raise ValueError(
            f"Annotation {annotation_id} uses COCO RLE segmentation. "
            "This script expects polygon segmentation from the COCO Segmentation export."
        )
    if not isinstance(segmentation, list):
        raise ValueError(f"Invalid segmentation for annotation {annotation_id}.")

    for polygon in segmentation:
        if len(polygon) < 6 or len(polygon) % 2:
            raise ValueError(
                f"Invalid polygon for annotation {annotation_id}; "
                "expected at least 3 (x, y) points."
            )
        points = [
            (round(x), round(y))
            for x, y in zip(polygon[0::2], polygon[1::2])
        ]
        mask_draw.polygon(points, fill=255)


def convert_split(split_dir):
    """Create one binary PNG mask per image in a split."""
    annotation_path = find_coco_annotations(split_dir)
    if annotation_path is None:
        print(f"Skipping {split_dir}: no COCO annotation JSON found.")
        return 0

    coco = json.loads(annotation_path.read_text())
    images_by_id = {image["id"]: image for image in coco.get("images", [])}
    annotations_by_image = {}
    for annotation in coco.get("annotations", []):
        annotations_by_image.setdefault(annotation["image_id"], []).append(annotation)

    mask_dir = split_dir / "masks"
    mask_dir.mkdir(parents=True, exist_ok=True)
    converted = 0
    missing_images = []

    for image_info in images_by_id.values():
        image_path = find_image(split_dir, image_info["file_name"])
        if image_path is None:
            missing_images.append(image_info["file_name"])
            continue

        with Image.open(image_path) as image:
            width, height = image.size

        mask = Image.new("L", (width, height), 0)
        mask_draw = ImageDraw.Draw(mask)
        for annotation in annotations_by_image.get(image_info["id"], []):
            draw_coco_polygons(
                mask_draw,
                annotation.get("segmentation", []),
                width,
                height,
                annotation.get("id", "unknown"),
            )

        mask.save(mask_dir / f"{image_path.stem}.png")
        converted += 1

    if missing_images:
        examples = "\n".join(missing_images[:10])
        raise FileNotFoundError(
            f"Could not find {len(missing_images)} image(s) referenced by "
            f"{annotation_path}. Examples:\n{examples}"
        )

    print(f"{split_dir.name}: created {converted} binary masks.")
    return converted


def main():
    total = 0
    for split in SPLITS:
        split_dir = DATASET_DIR / split
        if split_dir.is_dir():
            total += convert_split(split_dir)
        else:
            print(f"Skipping missing split folder: {split_dir}")

    print(f"Created {total} masks total.")
    print("Mask values: 0 = traversable, 255 = annotated non-traversable area.")


if __name__ == "__main__":
    main()
