#!/usr/bin/env python3
"""Export the trained grayscale U-Net++ checkpoint to ONNX and an RVC2 blob.

Install export dependencies: python3 -m pip install onnx blobconverter
Run: python3 export_model.py
Use --onnx-only to export locally without uploading to Luxonis BlobConverter.

Both models accept normalized grayscale NCHW input (1, 1, H, W), values
0..1, and output rock probabilities in the same shape. Apply the training
letterbox resize and divide pixels by 255 before inference. The blob expects
FP16 input; ONNX expects float32. Threshold probabilities at 0.5 for a mask.
"""

import argparse
from pathlib import Path
import sys


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--checkpoint", type=Path,
        default=Path(__file__).resolve().parent / "unetplusplus_synmars_best_50epoch.pth",
    )
    parser.add_argument(
        "--output-dir", type=Path,
        help="Output directory (default: exports beside the checkpoint)",
    )
    parser.add_argument("--width", type=int, help="Override checkpoint input width")
    parser.add_argument("--height", type=int, help="Override checkpoint input height")
    parser.add_argument("--shaves", type=int, choices=range(1, 17), default=6)
    parser.add_argument(
        "--openvino-version", choices=("2021.4", "2022.1", "2022.3_RVC3"),
        default="2022.1", help="2022.1 targets RVC2; 2022.3_RVC3 targets RVC3",
    )
    parser.add_argument("--onnx-only", action="store_true")
    return parser.parse_args()


def export(args):
    # Lazy imports keep --help usable before installing export dependencies.
    import onnx
    import segmentation_models_pytorch as smp
    import torch

    checkpoint_path = args.checkpoint.expanduser().resolve()
    if not checkpoint_path.is_file():
        raise FileNotFoundError(f"Checkpoint not found: {checkpoint_path}")
    checkpoint = torch.load(checkpoint_path, map_location="cpu", weights_only=True)
    width = args.width if args.width is not None else int(
        checkpoint.get("image_width", checkpoint.get("image_size", 640))
    )
    height = args.height if args.height is not None else int(
        checkpoint.get("image_height", checkpoint.get("image_size", 480))
    )
    if any(value < 32 or value % 32 for value in (width, height)):
        raise ValueError("Width and height must be positive multiples of 32 (at least 32).")
    model = smp.UnetPlusPlus(
        encoder_name="resnet34", encoder_weights=None,
        in_channels=1, classes=1, activation=None,
    )
    model.load_state_dict(checkpoint.get("model_state_dict", checkpoint), strict=True)
    # The ROS detector applies sigmoid outside the model; include it in exports.
    model = torch.nn.Sequential(model, torch.nn.Sigmoid()).eval()
    output_dir = (args.output_dir or checkpoint_path.parent / "exports").expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    onnx_path = output_dir / f"{checkpoint_path.stem}_{width}x{height}.onnx"
    dummy = torch.zeros(1, 1, height, width, dtype=torch.float32)
    print(f"Exporting {checkpoint_path.name}: input {tuple(dummy.shape)}", flush=True)
    with torch.inference_mode():
        torch.onnx.export(
            model, dummy, str(onnx_path), opset_version=12,
            input_names=["image"], output_names=["rock_probability"],
            dynamo=False, do_constant_folding=True,
        )
    onnx.checker.check_model(str(onnx_path))
    print(f"Validated ONNX: {onnx_path}", flush=True)
    if args.onnx_only:
        return

    import blobconverter

    print(
        "Uploading ONNX to Luxonis BlobConverter for FP16 compilation "
        f"(OpenVINO {args.openvino_version}, {args.shaves} SHAVEs)...",
        flush=True,
    )
    blob_path = blobconverter.from_onnx(
        model=str(onnx_path), output_dir=output_dir,
        data_type="FP16", shaves=args.shaves, version=args.openvino_version,
        # Disable BlobConverter's default RGB mean/scale preprocessing.
        optimizer_params=[], compile_params=["-ip FP16", "-op FP16"],
    )
    if not Path(blob_path).is_file():
        raise RuntimeError(f"BlobConverter did not produce a blob: {blob_path}")
    print(f"Blob: {blob_path}")


def main():
    args = parse_args()
    try:
        export(args)
    except ImportError as error:
        print(
            f"Missing dependency: {error}. Install with:\n"
            "python3 -m pip install torch segmentation-models-pytorch onnx blobconverter",
            file=sys.stderr,
        )
        return 1
    except Exception as error:
        print(f"Export failed: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
