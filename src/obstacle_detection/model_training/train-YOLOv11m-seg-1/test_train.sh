#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"

python3 train_unet.py \
    --data-dir ../YOLOv11m-seg-1-cleaned \
    --epochs 5 \
    --batch-size 2 \
    --max-train-samples 256 \
    --max-valid-samples 128 \
    --image-width 640 \
    --image-height 480 \
    --max-test-samples 128 \
    --sample-seed 67 \
    --encoder-weights none \
    --device auto \
    --output ./test_unet_256s_5e_clean.pth \
    "$@"
