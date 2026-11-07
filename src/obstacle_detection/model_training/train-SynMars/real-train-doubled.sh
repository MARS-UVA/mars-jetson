#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
cd "$SCRIPT_DIR"

python3 train_unet.py \
    --data-dir ./SynMars-doubled \
    --epochs 50 \
    --batch-size 4 \
    --image-size 512 \
    --learning-rate 0.0001 \
    --encoder-weights imagenet \
    --device auto \
    --output ./unetplusplus_synmars_doubled_best_50epoch.pth \
    "$@"
