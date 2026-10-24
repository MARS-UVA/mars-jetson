"""Run the trained binary rock segmentation model on the front camera."""

import sys
from pathlib import Path

import numpy as np
import rclpy
import segmentation_models_pytorch as smp
import torch
from PIL import Image as PILImage
from rclpy.node import Node
from sensor_msgs.msg import Image


DEFAULT_MODEL = (
    "/home/ws/src/obstacle_detection/model_training/train-YOLOv11m-seg-1/"
    "test_unet_256s_5e_clean.pth"
)


def resize_with_padding(image, width, height):
    """Match the training resize_pair preprocessing for a grayscale image."""
    scale = min(width / image.width, height / image.height)
    size = (max(1, round(image.width * scale)), max(1, round(image.height * scale)))
    resized = image.resize(size, PILImage.Resampling.BILINEAR)
    canvas = PILImage.new("L", (width, height), 0)
    offset = ((width - size[0]) // 2, (height - size[1]) // 2)
    canvas.paste(resized, offset)
    return canvas


class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__("obstacle_detection")
        self.declare_parameter("model_path", DEFAULT_MODEL)
        self.declare_parameter("input_topic", "/front_camera/image_raw")
        self.declare_parameter("threshold", 0.5)

        self.load_model()

        self.mask_publisher = self.create_publisher(Image, "/obstacles/mask", 10)
        self.confidence_publisher = self.create_publisher(Image, "/obstacles/confidence", 10)
        topic = str(self.get_parameter("input_topic").value)
        self.image_subscription = self.create_subscription(Image, topic, self.camera_ingest, 10)

    def load_model(self):
        """Load the checkpoint and prepare the model for inference."""
        model_path = Path(str(self.get_parameter("model_path").value)).expanduser()
        if not model_path.is_file():
            raise FileNotFoundError(f"Model checkpoint not found: {model_path}")

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        checkpoint = torch.load(model_path, map_location=self.device, weights_only=False)
        self.width = int(checkpoint.get("image_width", checkpoint.get("image_size", 640)))
        self.height = int(checkpoint.get("image_height", checkpoint.get("image_size", 480)))
        self.model = smp.UnetPlusPlus(
            encoder_name="resnet34",
            encoder_weights=None,
            in_channels=1,
            classes=1,
            activation=None,
        ).to(self.device)
        state = checkpoint.get("model_state_dict", checkpoint)
        self.model.load_state_dict(state)
        self.model.eval()
        self.threshold = float(str(self.get_parameter("threshold").value))
        self.get_logger().info(
            f"Loaded {model_path} on {self.device}; model input {self.width}x{self.height}"
        )

    def camera_ingest(self, message):
        if message.encoding != "mono8":
            self.get_logger().warning(
                f"Expected mono8 image, received {message.encoding}; skipping frame",
                throttle_duration_sec=5.0,
            )
            return

        image = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )[:, : message.width]
        source = PILImage.fromarray(image, mode="L")
        model_image = resize_with_padding(source, self.width, self.height)
        input_array = np.asarray(model_image, dtype=np.float32).copy() / 255.0
        tensor = torch.from_numpy(input_array).unsqueeze(0).unsqueeze(0).to(self.device)

        with torch.inference_mode():
            rock_probability = torch.sigmoid(self.model(tensor))[0, 0].cpu().numpy()

        # Training labels are 0=traversable and nonzero=non-traversable (rock).
        binary_mask = (rock_probability >= self.threshold).astype(np.uint8)
        class_confidence = np.where(
            binary_mask != 0, rock_probability, 1.0 - rock_probability
        )
        # Use class IDs 0 and 1 (rather than display intensities 0 and 255).
        mask_image = PILImage.fromarray(binary_mask, mode="L").resize(
            (message.width, message.height), PILImage.Resampling.NEAREST
        )
        confidence_image = PILImage.fromarray(
            np.clip(class_confidence * 255.0, 0, 255).astype(np.uint8), mode="L"
        ).resize((message.width, message.height), PILImage.Resampling.BILINEAR)

        self.mask_publisher.publish(self.make_image_message(message, mask_image, "mono8"))
        self.confidence_publisher.publish(
            self.make_image_message(message, confidence_image, "mono8")
        )

    @staticmethod
    def make_image_message(source, image, encoding):
        result = Image()
        result.header = source.header
        result.height = image.height
        result.width = image.width
        result.encoding = encoding
        result.is_bigendian = 0
        result.step = image.width
        result.data = np.asarray(image, dtype=np.uint8).tobytes()
        return result


def main() -> None:
    rclpy.init(args=sys.argv)
    node = ObstacleDetectionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
