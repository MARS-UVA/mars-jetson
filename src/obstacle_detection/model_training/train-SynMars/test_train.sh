#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"

python3 train_unet.py \
    --data-dir ./data \
    --epochs 5 \
    --batch-size 2 \
    --max-train-samples 256 \
    --max-valid-samples 128 \
    --max-test-samples 128 \
    --image-width 512 \
    --image-height 512 \
    --sample-seed 7 \
    --encoder-weights none \
    --device auto \
    --output ./test_unet_synmars.pth \
    "$@"
