#!/usr/bin/env python3
"""Create a filtered copy of the COCO-exported rock image/mask dataset.

Images are kept when their existing binary training mask has 1%–25%
foreground coverage (inclusive) and the COCO file lists at most 10 instances.
The source dataset is never modified.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import logging
import os
import shutil
import tempfile
from collections import defaultdict
from pathlib import Path

import cv2
import numpy as np

LOG = logging.getLogger("rock_dataset_filter")
HERE = Path(__file__).resolve().parent
DEFAULT_SOURCE = HERE.parent / "YOLOv11m-seg-1"
DEFAULT_OUTPUT = HERE.parent / "YOLOv11m-seg-1-cleaned"
SPLITS = ("train", "valid", "test")
IMAGE_EXTENSIONS = {".jpg", ".jpeg", ".png", ".bmp", ".tif", ".tiff"}


def _signature(root: Path) -> str:
    digest = hashlib.sha256()
    for path in sorted(p for p in root.rglob("*") if p.is_file()):
        if path.is_symlink():
            raise ValueError(f"Symbolic links are unsupported in the dataset: {path}")
        if any(part.startswith(".") for part in path.relative_to(root).parts):
            continue
        stat = path.stat()
        digest.update(path.relative_to(root).as_posix().encode())
        digest.update(f"\0{stat.st_size}\0{stat.st_mtime_ns}\n".encode())
    return digest.hexdigest()


def _resolve_image(split_dir: Path, file_name: str) -> Path:
    relative = Path(file_name)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError(f"Unsafe COCO image path: {file_name}")
    candidates = (split_dir / relative, split_dir / "images" / relative, split_dir / relative.name, split_dir / "images" / relative.name)
    for candidate in candidates:
        if candidate.is_file() and candidate.suffix.lower() in IMAGE_EXTENSIONS:
            resolved = candidate.resolve()
            if not resolved.is_relative_to(split_dir.resolve()):
                raise ValueError(f"COCO image path escapes its split: {file_name}")
            return resolved
    raise FileNotFoundError(f"Image referenced by COCO JSON is missing: {split_dir / file_name}")


def _load_coco(split_dir: Path) -> tuple[Path, dict]:
    annotation_file = split_dir / "_annotations.coco.json"
    if not annotation_file.is_file():
        other = sorted(split_dir.glob("*.json"))
        if len(other) != 1:
            raise FileNotFoundError(f"Expected _annotations.coco.json in {split_dir}")
        annotation_file = other[0]
    data = json.loads(annotation_file.read_text(encoding="utf-8"))
    if not isinstance(data.get("images"), list) or not isinstance(data.get("annotations"), list):
        raise ValueError(f"Invalid COCO images/annotations arrays in {annotation_file}")
    return annotation_file, data


def _evaluate_image(image_info: dict, annotations: list[dict], split_dir: Path, min_density: float, max_density: float, max_rocks: int) -> dict:
    result = {
        "image_id": image_info.get("id"), "image": str(image_info.get("file_name", "")),
        "instances": len(annotations), "foreground_pixels": "", "density": "",
        "accepted": False, "reason": "",
    }
    try:
        image_path = _resolve_image(split_dir, str(image_info["file_name"]))
        mask_path = split_dir / "masks" / f"{image_path.stem}.png"
        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        mask = cv2.imread(str(mask_path), cv2.IMREAD_GRAYSCALE)
        if image is None or image.size == 0:
            raise ValueError("corrupt_image")
        if mask is None or mask.size == 0:
            raise ValueError("missing_or_corrupt_mask")
        height, width = image.shape[:2]
        if mask.shape != (height, width):
            raise ValueError(f"image_mask_dimension_mismatch:{width}x{height}_vs_{mask.shape[1]}x{mask.shape[0]}")
        declared_width, declared_height = int(image_info.get("width", width)), int(image_info.get("height", height))
        if (declared_width, declared_height) != (width, height):
            raise ValueError(f"coco_image_dimension_mismatch:{declared_width}x{declared_height}_vs_{width}x{height}")
        foreground_pixels = int(np.count_nonzero(mask))
        density = foreground_pixels / float(width * height)
        reasons = []
        if len(annotations) > max_rocks:
            reasons.append("rock_count_exceeds_maximum")
        if density < min_density:
            reasons.append("rock_density_below_minimum")
        if density > max_density:
            reasons.append("rock_density_above_maximum")
        result.update({
            "foreground_pixels": foreground_pixels, "density": density,
            "accepted": not bool(reasons), "reason": ";".join(reasons),
            "image_path": image_path, "mask_path": mask_path,
        })
    except (KeyError, OSError, ValueError, cv2.error) as exc:
        result["reason"] = str(exc)
    return result


def clean_dataset(args) -> dict:
    source = args.source.expanduser().resolve()
    output = args.output.expanduser().resolve()
    if not source.is_dir():
        raise FileNotFoundError(f"Source dataset not found: {source}")
    if source == output or source in output.parents or output in source.parents:
        raise ValueError("Source and output must be separate, non-nested directories")
    if not (0 <= args.min_density <= args.max_density <= 1):
        raise ValueError("Density thresholds must satisfy 0 <= min <= max <= 1")
    if args.max_rocks < 0:
        raise ValueError("--max-rocks must be nonnegative")

    split_data = {}
    for split in SPLITS:
        split_dir = source / split
        if split_dir.is_dir():
            annotation_file, coco = _load_coco(split_dir)
            split_data[split] = (split_dir, annotation_file, coco)
    if not split_data:
        raise FileNotFoundError(f"No train/valid/test COCO splits found under {source}")

    signature = _signature(source)
    settings = {
        "source": str(source), "source_signature": signature,
        "min_density": args.min_density, "max_density": args.max_density,
        "max_rocks": args.max_rocks,
    }
    if output.exists():
        report_file = output / "cleaning_report.json"
        if report_file.is_file():
            existing = json.loads(report_file.read_text(encoding="utf-8"))
            if all(existing.get(key) == value for key, value in settings.items()):
                LOG.info("Existing output matches this source and filter; leaving it unchanged")
                return existing
        raise FileExistsError(f"Output already exists and does not match this run: {output}")

    output.parent.mkdir(parents=True, exist_ok=True)
    temp_root = Path(tempfile.mkdtemp(prefix=f".{output.name}.", dir=output.parent))
    manifest_rows = []
    split_summaries = {}
    try:
        for split, (split_dir, annotation_file, coco) in split_data.items():
            images = coco["images"]
            image_ids = [image.get("id") for image in images]
            if len(set(image_ids)) != len(image_ids):
                raise ValueError(f"Duplicate COCO image IDs in {annotation_file}")
            annotations_by_image: dict[object, list[dict]] = defaultdict(list)
            image_id_set = set(image_ids)
            orphan_annotations = 0
            for annotation in coco["annotations"]:
                image_id = annotation.get("image_id")
                if image_id in image_id_set:
                    annotations_by_image[image_id].append(annotation)
                else:
                    orphan_annotations += 1

            accepted_ids = set()
            accepted_images = []
            accepted_annotations = []
            split_rows = []
            for image_info in images:
                image_annotations = annotations_by_image.get(image_info.get("id"), [])
                row = _evaluate_image(image_info, image_annotations, split_dir, args.min_density, args.max_density, args.max_rocks)
                row["split"] = split
                manifest_rows.append(row)
                split_rows.append(row)
                if not row["accepted"]:
                    continue
                accepted_ids.add(image_info["id"])
                image_path = row.pop("image_path")
                mask_path = row.pop("mask_path")
                relative_image = image_path.relative_to(split_dir)
                output_split = temp_root / split
                output_image = output_split / relative_image
                output_mask = output_split / "masks" / f"{image_path.stem}.png"
                output_image.parent.mkdir(parents=True, exist_ok=True)
                output_mask.parent.mkdir(parents=True, exist_ok=True)
                shutil.copy2(image_path, output_image)
                shutil.copy2(mask_path, output_mask)
                updated_info = dict(image_info)
                updated_info["file_name"] = relative_image.as_posix()
                accepted_images.append(updated_info)
                accepted_annotations.extend(image_annotations)

            output_split = temp_root / split
            output_split.mkdir(parents=True, exist_ok=True)
            filtered_coco = dict(coco)
            filtered_coco["images"] = accepted_images
            filtered_coco["annotations"] = accepted_annotations
            (output_split / annotation_file.name).write_text(json.dumps(filtered_coco, indent=2) + "\n", encoding="utf-8")
            reasons: dict[str, int] = {}
            for row in split_rows:
                if not row["accepted"]:
                    for reason in str(row["reason"]).split(";"):
                        if reason:
                            reasons[reason] = reasons.get(reason, 0) + 1
            split_summaries[split] = {
                "source_images": len(images), "accepted_images": len(accepted_images),
                "rejected_images": len(images) - len(accepted_images),
                "accepted_annotations": len(accepted_annotations),
                "orphan_annotations_removed": orphan_annotations,
                "reason_counts": reasons,
            }

        # Copy only dataset-level documentation/metadata, not unlisted images.
        for path in source.iterdir():
            if path.is_file() and path.name not in {".DS_Store"}:
                shutil.copy2(path, temp_root / path.name)

        report = {
            **settings, "output": str(output),
            "density_definition": "nonzero pixels in the existing training mask divided by full image area",
            "instance_definition": "number of COCO annotations for the image; all annotations are counted because the existing mask converter renders every category as foreground",
            "thresholds_inclusive": True,
            "counts": {
                "source_images": sum(v["source_images"] for v in split_summaries.values()),
                "accepted_images": sum(v["accepted_images"] for v in split_summaries.values()),
                "rejected_images": sum(v["rejected_images"] for v in split_summaries.values()),
                "splits": split_summaries,
            },
        }
        with (temp_root / "cleaning_manifest.csv").open("w", newline="", encoding="utf-8") as stream:
            fields = ("split", "image_id", "image", "instances", "foreground_pixels", "density", "accepted", "reason")
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for row in manifest_rows:
                writer.writerow({key: row.get(key, "") for key in fields})
        (temp_root / "cleaning_report.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        temp_root.chmod(source.stat().st_mode & 0o777)
        os.replace(temp_root, output)
        LOG.info("Kept %d/%d images in %s", report["counts"]["accepted_images"], report["counts"]["source_images"], output)
        return report
    except Exception:
        shutil.rmtree(temp_root, ignore_errors=True)
        raise


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE, help=f"source COCO dataset (default: {DEFAULT_SOURCE})")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT, help=f"filtered dataset output (default: {DEFAULT_OUTPUT})")
    parser.add_argument("--min-density", type=float, default=0.01, help="minimum fraction of mask pixels (default: 0.01)")
    parser.add_argument("--max-density", type=float, default=0.25, help="maximum fraction of mask pixels (default: 0.25)")
    parser.add_argument("--max-rocks", type=int, default=10, help="maximum COCO annotations per image (default: 10)")
    return parser


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    args = build_parser().parse_args()
    try:
        report = clean_dataset(args)
    except Exception as exc:
        LOG.error("%s", exc)
        return 2
    print(json.dumps(report, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
