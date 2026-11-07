#!/usr/bin/env python3
"""Preview export_model.py's FP16 segmentation blob on an RVC2 OAK-D.

Install: python3 -m pip install 'depthai>=2.24,<3' opencv-python Pillow numpy
Run: python3 view_oak_mask.py --blob exports/your_model.blob
The host preprocesses images; the OAK runs inference. Press Q/Escape to exit.
"""

import argparse
from pathlib import Path
import time


def prepare_image(frame, width, height):
    """Match the PIL grayscale and letterbox processing used by the ROS node."""
    import numpy as np
    from PIL import Image

    source = Image.fromarray(frame[:, :, ::-1]).convert("L")
    scale = min(width / source.width, height / source.height)
    resized_width = max(1, round(source.width * scale))
    resized_height = max(1, round(source.height * scale))
    left = (width - resized_width) // 2
    top = (height - resized_height) // 2
    canvas = Image.new("L", (width, height), 0)
    canvas.paste(source.resize(
        (resized_width, resized_height), Image.Resampling.BILINEAR
    ), (left, top))
    tensor = np.asarray(canvas, dtype=np.float32) / 255.0
    return tensor.astype(np.float16), (left, top, resized_width, resized_height)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--blob", type=Path, required=True)
    parser.add_argument("--threshold", type=float, default=0.5)
    args = parser.parse_args()
    if not args.blob.is_file():
        parser.error(f"Blob not found: {args.blob}")
    if not 0 <= args.threshold <= 1:
        parser.error("Threshold must be between 0 and 1")

    import cv2
    import depthai as dai
    import numpy as np
    from PIL import Image

    if not dai.__version__.startswith("2."):
        parser.error("This viewer uses DepthAI v2: pip install 'depthai>=2.24,<3'")
    blob = dai.OpenVINO.Blob(str(args.blob.resolve()))
    if len(blob.networkInputs) != 1 or len(blob.networkOutputs) != 1:
        parser.error("Expected one grayscale input and one probability output")
    input_name, input_info = next(iter(blob.networkInputs.items()))
    output_name = next(iter(blob.networkOutputs))
    # OpenVINO blob dimensions are stored W,H,C,N (reverse of NCHW).
    dims = list(input_info.dims)
    if len(dims) != 4 or dims[2:] != [1, 1]:
        parser.error(f"Expected a 1x1xHxW input, got blob dimensions {dims}")
    width, height = dims[:2]
    print(f"Input {input_name}: 1x1x{height}x{width}; output: {output_name}")

    pipeline = dai.Pipeline()
    pipeline.setOpenVINOVersion(blob.version)
    camera = pipeline.create(dai.node.ColorCamera)
    camera.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    camera.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    camera.setPreviewSize(640, 480)
    camera.setPreviewKeepAspectRatio(True)
    camera.setInterleaved(False)
    camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
    camera.setFps(15)
    camera_out = pipeline.create(dai.node.XLinkOut)
    camera_out.setStreamName("camera")
    camera.preview.link(camera_out.input)
    camera_out.input.setBlocking(False)
    camera_out.input.setQueueSize(1)

    network = pipeline.create(dai.node.NeuralNetwork)
    network.setBlob(blob)
    network.setNumInferenceThreads(1)
    host_in = pipeline.create(dai.node.XLinkIn)
    host_in.setStreamName("nn_input")
    host_in.out.link(network.input)
    host_out = pipeline.create(dai.node.XLinkOut)
    host_out.setStreamName("nn_output")
    network.out.link(host_out.input)

    window = "OAK-D: image | rock overlay | mask"
    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.createTrackbar("Threshold %", window, round(args.threshold * 100), 100, lambda _: None)
    print("Red = predicted rock. Adjust threshold; Q/Escape exits.")
    try:
        with dai.Device(pipeline) as device:
            camera_queue = device.getOutputQueue("camera", maxSize=1, blocking=False)
            input_queue = device.getInputQueue("nn_input")
            output_queue = device.getOutputQueue("nn_output", maxSize=1, blocking=True)
            pending = None
            while True:
                if cv2.waitKey(1) & 0xFF in (27, ord("q")):
                    break
                if pending is None:
                    message = camera_queue.tryGet()
                    if message is None:
                        time.sleep(0.001)
                        continue
                    frame = message.getCvFrame()
                    tensor, bounds = prepare_image(frame, width, height)
                    data = dai.NNData()
                    # A list of floats selects NNData's FP16 overload.
                    data.setLayer(input_name, tensor.ravel().tolist())
                    started = time.monotonic()
                    input_queue.send(data)
                    pending = (frame, bounds, started)
                result = output_queue.tryGet()
                if result is None:
                    if time.monotonic() - pending[2] > 30:
                        raise RuntimeError("No inference result after 30 seconds")
                    time.sleep(0.001)
                    continue
                frame, (left, top, content_w, content_h), started = pending
                elapsed = time.monotonic() - started
                probability = np.asarray(result.getLayerFp16(output_name), dtype=np.float32)
                if probability.size != width * height:
                    raise RuntimeError(f"Unexpected output size: {probability.size}")
                probability = probability.reshape(height, width)
                probability = probability[top:top + content_h, left:left + content_w]
                probability = np.asarray(Image.fromarray(probability).resize(
                    (frame.shape[1], frame.shape[0]), Image.Resampling.BILINEAR
                ))
                threshold = cv2.getTrackbarPos("Threshold %", window) / 100.0
                mask = probability >= threshold
                overlay = frame.copy()
                overlay[mask] = (
                    frame[mask].astype(np.float32) * 0.5 + np.array([0, 0, 255]) * 0.5
                ).astype(np.uint8)
                mask_view = cv2.cvtColor(mask.astype(np.uint8) * 255, cv2.COLOR_GRAY2BGR)
                display = np.hstack((frame, overlay, mask_view))
                cv2.putText(display, f"Inference round trip: {elapsed * 1000:.0f} ms | threshold {threshold:.2f}",
                            (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)
                cv2.imshow(window, display)
                pending = None
    finally:
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
