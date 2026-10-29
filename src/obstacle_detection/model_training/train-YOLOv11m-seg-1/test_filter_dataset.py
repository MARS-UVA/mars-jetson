"""Tests for COCO image/mask filtering by foreground coverage and instance count."""

import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace

import cv2
import numpy as np

import filter_dataset


class FilterDatasetTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory(prefix=".filter-test-", dir=filter_dataset.HERE)
        self.root = Path(self.temp.name)
        self.source = self.root / "YOLOv11m-seg-1"
        self.output = self.root / "YOLOv11m-seg-1-cleaned"
        self.source.mkdir()
        (self.source / "README.roboflow.txt").write_text("dataset fixture\n")

    def tearDown(self):
        self.temp.cleanup()

    def add_split(self, split, cases):
        split_dir = self.source / split
        (split_dir / "masks").mkdir(parents=True)
        images, annotations = [], []
        for index, (name, mask, annotation_count) in enumerate(cases):
            image = np.full((100, 100, 3), 80, dtype=np.uint8)
            image_path = split_dir / f"{name}.jpg"
            mask_path = split_dir / "masks" / f"{name}.png"
            self.assertTrue(cv2.imwrite(str(image_path), image))
            self.assertTrue(cv2.imwrite(str(mask_path), mask))
            image_id = index + 1
            images.append({"id": image_id, "file_name": image_path.name, "width": 100, "height": 100})
            for instance in range(annotation_count):
                annotations.append({"id": len(annotations) + 1, "image_id": image_id, "category_id": 2, "bbox": [], "area": 0, "segmentation": [[]]})
        coco = {"info": {"fixture": True}, "categories": [{"id": 2, "name": "rock"}], "images": images, "annotations": annotations}
        (split_dir / "_annotations.coco.json").write_text(json.dumps(coco), encoding="utf-8")

    @staticmethod
    def mask_pixels(count):
        mask = np.zeros((100, 100), dtype=np.uint8)
        mask.flat[:count] = 255
        return mask

    def args(self):
        return SimpleNamespace(source=self.source, output=self.output, min_density=0.01, max_density=0.25, max_rocks=10)

    def test_boundaries_count_and_coco_filtering(self):
        self.add_split("train", [
            ("one_percent", self.mask_pixels(100), 1),
            ("twentyfive_percent", self.mask_pixels(2500), 10),
            ("ten_rocks", self.mask_pixels(1000), 10),
            ("below_min", self.mask_pixels(99), 1),
            ("above_max", self.mask_pixels(2501), 1),
            ("eleven_rocks", self.mask_pixels(1100), 11),
        ])
        self.add_split("valid", [("valid_kept", self.mask_pixels(500), 2)])
        self.add_split("test", [("test_kept", self.mask_pixels(800), 3)])
        report = filter_dataset.clean_dataset(self.args())
        self.assertEqual(report["counts"]["source_images"], 8)
        self.assertEqual(report["counts"]["accepted_images"], 5)
        self.assertEqual(report["counts"]["rejected_images"], 3)
        self.assertTrue((self.output / "README.roboflow.txt").is_file())
        for split, name in (("train", "one_percent"), ("train", "twentyfive_percent"), ("train", "ten_rocks"), ("valid", "valid_kept"), ("test", "test_kept")):
            self.assertTrue((self.output / split / f"{name}.jpg").is_file())
            self.assertTrue((self.output / split / "masks" / f"{name}.png").is_file())
        self.assertFalse((self.output / "train/below_min.jpg").exists())
        self.assertFalse((self.output / "train/above_max.jpg").exists())
        self.assertFalse((self.output / "train/eleven_rocks.jpg").exists())
        cleaned = json.loads((self.output / "train/_annotations.coco.json").read_text())
        self.assertEqual([item["file_name"] for item in cleaned["images"]], ["one_percent.jpg", "twentyfive_percent.jpg", "ten_rocks.jpg"])
        self.assertEqual(len(cleaned["annotations"]), 21)

    def test_rerun_with_same_inputs_is_noop(self):
        self.add_split("train", [("kept", self.mask_pixels(100), 1)])
        first = filter_dataset.clean_dataset(self.args())
        second = filter_dataset.clean_dataset(self.args())
        self.assertEqual(first, second)


if __name__ == "__main__":
    unittest.main()
