"""Prepare SynMars-doubled with mirrored, brightness-inverted training copies.

Reads the same route archives as prepare_synmars.py. Each training image keeps
its original and gains one horizontally mirrored, brightness-inverted copy.
The corresponding mask is mirrored only: rock/ground labels never invert.
Validation and test splits remain unaugmented for realistic evaluation.

Run: python3 prepare_synmars2.py
Train: python3 train_unet.py --data-dir SynMars-doubled
"""

import argparse
import csv
from pathlib import Path
import shutil

from PIL import Image, ImageOps

from prepare_synmars import DEFAULT_SOURCE, ROUTE_SPLITS, SCRIPT_DIR, prepare_dataset


DEFAULT_OUTPUT = SCRIPT_DIR / "SynMars-doubled"


def prepare_doubled_dataset(source, output):
    """Prepare route-separated originals, then double only the training split."""
    output = output.expanduser().resolve()
    # Reuse archive validation, stereo splitting, grayscale conversion, and
    # binary mask conversion. This also refuses to overwrite existing output.
    counts, pairs, manifest_path = prepare_dataset(source, output, ROUTE_SPLITS)
    try:
        with manifest_path.open(newline="", encoding="utf-8") as stream:
            reader = csv.DictReader(stream)
            fieldnames = list(reader.fieldnames) + ["augmentation"]
            rows = [dict(row, augmentation="original") for row in reader]

        augmented_rows = []
        for row in rows:
            image_path = Path(row["image"])
            if image_path.parent.parent != output / "train":
                continue
            mask_path = Path(row["mask"])
            suffix = "_mirror_inverted"
            augmented_image = image_path.with_stem(image_path.stem + suffix)
            augmented_mask = mask_path.with_stem(mask_path.stem + suffix)
            with Image.open(image_path) as image:
                ImageOps.invert(ImageOps.mirror(image.convert("L"))).save(augmented_image)
            with Image.open(mask_path) as mask:
                ImageOps.mirror(mask.convert("L")).save(augmented_mask)
            augmented_rows.append(dict(
                row, image=str(augmented_image), mask=str(augmented_mask),
                augmentation="mirror_inverted",
            ))

        with manifest_path.open("w", newline="", encoding="utf-8") as stream:
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            writer.writeheader()
            writer.writerows(rows)
            writer.writerows(augmented_rows)
        counts["train"] += len(augmented_rows)
        return counts, pairs + len(augmented_rows), manifest_path
    except Exception:
        # Only remove the new output created successfully by prepare_dataset.
        shutil.rmtree(output, ignore_errors=True)
        raise


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT)
    args = parser.parse_args()
    counts, pairs, manifest = prepare_doubled_dataset(args.source, args.output)
    print(f"Prepared {pairs} image/mask pairs.")
    for split, count in counts.items():
        print(f"  {split}: {count} pairs")
    print("Training doubled; validation and test kept original.")
    print(f"Manifest: {manifest}")
    print(f"Dataset: {args.output.expanduser().resolve()}")


if __name__ == "__main__":
    main()
