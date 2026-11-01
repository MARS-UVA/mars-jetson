# SynMars grayscale rock segmentation

This folder adapts the U-Net++ training, evaluation, and prediction comparison
workflow from `train-YOLOv11m-seg-1` for the SynMars image and mask archives.
It intentionally omits the dataset cleaning pipeline.

## Dataset source

The cloned repository is at `../SynMars`. Its checked-in `route1` through
`route6` archives contain the published route samples (25 frames per route),
not the separately linked SynMars-8k release. The upstream README links the
larger SynMars-8k archive (8,000 train, 1,000 validation, and 1,000 test
images) if you want to fetch that data separately.

The route archives pair `rgbNNNN.png` frames with `maskNNNN.png` masks. The
preparation script splits each 1920x1080 image and mask into the left and right
960x1080 views described by SynMars, converts only the camera images to
grayscale, and converts every non-black mask pixel to the binary rock class.
It keeps source resolution; training applies the same aspect-preserving resize
and padding used by the existing trainer.

Routes are split as groups to reduce adjacent-route leakage:

- Train: routes 1–4
- Validation: route 5
- Test: route 6

This yields only 300 image/mask pairs from the checked-in samples. For a useful
training run, obtain the separately hosted SynMars-8k data and arrange it in
the same `train/images`, `train/masks`, `valid/images`, `valid/masks`,
`test/images`, `test/masks` layout; then pass that directory with `--data-dir`.

## Prepare the checked-in route samples

```bash
python3 prepare_synmars.py
```

The output is written to `data/`. Existing output is never overwritten; move or
remove `data/` before preparing it again. `dataset_manifest.csv` records each
route, frame, view, image, and mask pair.

## Train and evaluate

```bash
./test_train.sh
./real_train.sh --device mps
```

The smoke script caps dataset samples and disables ImageNet encoder weights.
The full script defaults to 50 epochs and ImageNet encoder weights. Both select
CUDA, then MPS, then CPU with `--device auto`.

Evaluate a checkpoint and write predicted masks:

```bash
python3 evaluate_unet.py \
  --checkpoint ./unetplusplus_synmars_best.pth \
  --data-dir ./data \
  --split test
```

Create a comparison sheet from the test images, labels, and predictions:

```bash
python3 compare_images_and_masks.py \
  --data-dir ./data \
  --split test \
  --predictions-dir ./predictions
```

Install dependencies with `python3 -m pip install -r requirements.txt`.
