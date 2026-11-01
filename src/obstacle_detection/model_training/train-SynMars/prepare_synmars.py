"""Prepare the SynMars route archives as grayscale binary-mask splits."""

import argparse
import csv
import io
import re
import zipfile
from pathlib import Path

import numpy as np
from PIL import Image


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_SOURCE = SCRIPT_DIR.parent / "SynMars"
DEFAULT_OUTPUT = SCRIPT_DIR / "data"
ROUTE_SPLITS = {
    "train": ("route1", "route2", "route3", "route4"),
    "valid": ("route5",),
    "test": ("route6",),
}
IMAGE_PATTERN = re.compile(r"^rgb(\d+)\.png$", re.IGNORECASE)
MASK_PATTERN = re.compile(r"^mask(\d+)\.png$", re.IGNORECASE)


def archive_members(archive, pattern):
    """Index image or mask PNG members by their shared numeric frame ID."""
    found = {}
    for name in archive.namelist():
        match = pattern.fullmatch(Path(name).name)
        if match:
            frame_id = match.group(1)
            if frame_id in found:
                raise ValueError(f"Duplicate frame {frame_id} in {archive.filename}")
            found[frame_id] = name
    return found


def open_rgb(archive, member):
    """Read one image from a zip archive and detach it from the archive stream."""
    with Image.open(io.BytesIO(archive.read(member))) as image:
        return image.convert("RGB")


def save_pair(image_archive, mask_archive, image_member, mask_member,
              split_dir, route, frame_id, manifest):
    image = open_rgb(image_archive, image_member)
    mask = open_rgb(mask_archive, mask_member)
    if image.size != mask.size:
        raise ValueError(
            f"Image/mask size mismatch for {route} frame {frame_id}: "
            f"{image.size} vs {mask.size}"
        )
    if image.width % 2:
        raise ValueError(f"Expected an even image width in {route} frame {frame_id}")

    middle = image.width // 2
    left_box = (0, 0, middle, image.height)
    right_box = (middle, 0, image.width, image.height)
    for view_name, box in (("left", left_box), ("right", right_box)):
        image_view = image.crop(box).convert("L")
        mask_view = mask.crop(box)
        mask_array = np.asarray(mask_view, dtype=np.uint8)
        binary_mask = np.any(mask_array != 0, axis=2).astype(np.uint8) * 255

        stem = f"{route}_{frame_id}_{view_name}"
        image_path = split_dir / "images" / f"{stem}.png"
        mask_path = split_dir / "masks" / f"{stem}.png"
        image_path.parent.mkdir(parents=True, exist_ok=True)
        mask_path.parent.mkdir(parents=True, exist_ok=True)
        image_view.save(image_path)
        Image.fromarray(binary_mask, mode="L").save(mask_path)
        manifest.append((route, frame_id, view_name, str(image_path), str(mask_path)))


def prepare_dataset(source, output, route_splits):
    source = source.expanduser().resolve()
    output = output.expanduser().resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"SynMars clone not found: {source}")
    if output.exists():
        raise FileExistsError(
            f"Output already exists: {output}. Move or remove it before preparing again."
        )

    assigned_routes = [route for routes in route_splits.values() for route in routes]
    if len(assigned_routes) != len(set(assigned_routes)):
        raise ValueError("Each route may appear in only one split")

    output.mkdir(parents=True)
    manifest_rows = []
    counts = {split: 0 for split in route_splits}
    try:
        for split, routes in route_splits.items():
            for route in routes:
                image_zip = source / route / "img.zip"
                mask_zip = source / route / "mask.zip"
                if not image_zip.is_file() or not mask_zip.is_file():
                    raise FileNotFoundError(
                        f"Expected both img.zip and mask.zip under {source / route}"
                    )
                with zipfile.ZipFile(image_zip) as images, zipfile.ZipFile(mask_zip) as masks:
                    image_members = archive_members(images, IMAGE_PATTERN)
                    mask_members = archive_members(masks, MASK_PATTERN)
                    if not image_members:
                        raise ValueError(f"No rgb*.png frames found in {image_zip}")
                    if image_members.keys() != mask_members.keys():
                        only_images = sorted(image_members.keys() - mask_members.keys())
                        only_masks = sorted(mask_members.keys() - image_members.keys())
                        raise ValueError(
                            f"Unpaired frames in {route}: images without masks "
                            f"{only_images[:5]}, masks without images {only_masks[:5]}"
                        )

                    for frame_id in sorted(image_members, key=int):
                        before = len(manifest_rows)
                        save_pair(
                            images, masks,
                            image_members[frame_id], mask_members[frame_id],
                            output / split, route, frame_id, manifest_rows,
                        )
                        counts[split] += len(manifest_rows) - before

        manifest_path = output / "dataset_manifest.csv"
        with manifest_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(("route", "frame_id", "view", "image", "mask"))
            writer.writerows(manifest_rows)
    except Exception:
        import shutil

        shutil.rmtree(output, ignore_errors=True)
        raise

    return counts, len(manifest_rows), manifest_path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()

    counts, pairs, manifest = prepare_dataset(args.source, args.output, ROUTE_SPLITS)
    print(f"Prepared {pairs} image/mask pairs from SynMars route archives.")
    for split, count in counts.items():
        print(f"  {split}: {count} pairs")
    print(f"Manifest: {manifest}")
    print(f"Dataset: {args.output}")


if __name__ == "__main__":
    main()
