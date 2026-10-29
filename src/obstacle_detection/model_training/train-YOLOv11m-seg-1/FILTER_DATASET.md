# Filter the COCO image/mask dataset

Run from this directory:

```bash
python3 filter_dataset.py
```

The script reads the sibling `../YOLOv11m-seg-1` dataset and writes a filtered
copy to `../YOLOv11m-seg-1-cleaned`. For each COCO image it uses the exact
binary mask consumed by `train_unet.py` and counts the COCO instances. It keeps
an image when foreground coverage is between 1% and 25% inclusive and the
instance count is at most 10. The existing converter paints every COCO
category into the binary mask, so all COCO annotations for an image count as
rocks for this filter.

Images, masks, and each split's COCO JSON are filtered together. Dataset-level
Roboflow documentation and category metadata are preserved. The source is
never modified. The output contains `cleaning_manifest.csv` and
`cleaning_report.json`; repeating an identical run is a no-op. A non-matching
existing output causes an error instead of overwriting it.

To train on the cleaned copy, pass it explicitly:

```bash
python3 train_unet.py --data-dir ../YOLOv11m-seg-1-cleaned
```
