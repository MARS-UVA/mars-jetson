#!/bin/sh
set -eu

SCRIPT_DIR=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
REPO_DIR=$(dirname -- "$SCRIPT_DIR")
cd "$REPO_DIR"

python3 transform_train/train_unet.py \
    --epochs 50 \
    --batch-size 4 \
    --image-size 512 \
    --learning-rate 0.0001 \
    --encoder-weights imagenet \
    --output transform_train/unetplusplus_best.pth \
    "$@"
