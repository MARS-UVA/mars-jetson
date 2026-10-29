"""Run the trained binary rock segmentation model on an RGB-D camera image."""

import sys
from pathlib import Path

import numpy as np
import rclpy
import segmentation_models_pytorch as smp
import torch
from PIL import Image as PILImage
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image
from vision_msgs.msg import LabelInfo, VisionClass


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
    return canvas, (offset[0], offset[1], size[0], size[1])


def restore_source_scale(model_output, source_size, content_bounds):
    """Remove letterbox padding and resize a model output to the camera frame."""
    left, top, content_width, content_height = content_bounds
    content = model_output[
        top : top + content_height,
        left : left + content_width,
    ]
    return np.asarray(
        PILImage.fromarray(content.astype(np.float32), mode="F").resize(
            source_size, PILImage.Resampling.BILINEAR
        ),
        dtype=np.float32,
    )


class ObstacleDetectionNode(Node):
    def __init__(self):
        super().__init__("obstacle_detection")
        self.declare_parameter("model_path", DEFAULT_MODEL)
        self.declare_parameter("blue_input_topic", "/rgbd_blue/color/image_raw")
        self.declare_parameter("orange_input_topic", "/rgbd_orange/color/image_raw")
        self.declare_parameter("obstacle_threshold", 0.5)

        self.load_model()

        self.mask_publishers = {}
        self.confidence_publishers = {}
        for camera in ("blue", "orange"):
            self.mask_publishers[camera] = self.create_publisher(
                Image, f"/obstacles/{camera}/mask", 10
            )
            self.confidence_publishers[camera] = self.create_publisher(
                Image, f"/obstacles/{camera}/confidence", 10
            )
        labels_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.labels_publisher = self.create_publisher(
            LabelInfo, "/obstacles/label_info", labels_qos
        )
        labels = LabelInfo()
        labels.class_map = [
            VisionClass(class_id=0, class_name="traversable"),
            VisionClass(class_id=1, class_name="rock"),
        ]
        labels.threshold = self.threshold
        self.labels_publisher.publish(labels)
        self.image_subscriptions = []
        for camera in ("blue", "orange"):
            topic = str(self.get_parameter(f"{camera}_input_topic").value)
            self.image_subscriptions.append(
                self.create_subscription(
                    Image,
                    topic,
                    lambda message, camera=camera: self.camera_ingest(message, camera),
                    qos_profile_sensor_data,
                )
            )

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
        self.threshold = float(str(self.get_parameter("obstacle_threshold").value))
        self.get_logger().info(
            f"Loaded {model_path} on {self.device}; model input {self.width}x{self.height}"
        )

    def camera_ingest(self, message, camera):
        channels_by_encoding = {"mono8": 1, "rgb8": 3, "bgr8": 3}
        channels = channels_by_encoding.get(message.encoding)
        if channels is None:
            self.get_logger().warning(
                f"Unsupported image encoding {message.encoding}; "
                "expected mono8, rgb8, or bgr8",
                throttle_duration_sec=5.0,
            )
            return

        row = np.frombuffer(message.data, dtype=np.uint8).reshape(
            message.height, message.step
        )
        image = row[:, : message.width * channels]
        if channels == 1:
            source = PILImage.fromarray(
                image.reshape(message.height, message.width), mode="L"
            )
        else:
            image = image.reshape(message.height, message.width, channels)
            if message.encoding == "bgr8":
                image = image[:, :, ::-1]
            source = PILImage.fromarray(image, mode="RGB").convert("L")
        model_image, content_bounds = resize_with_padding(source, self.width, self.height)
        input_array = np.asarray(model_image, dtype=np.float32).copy() / 255.0
        tensor = torch.from_numpy(input_array).unsqueeze(0).unsqueeze(0).to(self.device)

        with torch.inference_mode():
            rock_probability = torch.sigmoid(self.model(tensor))[0, 0].cpu().numpy()

        rock_probability = restore_source_scale(
            rock_probability,
            (message.width, message.height),
            content_bounds,
        )
        # Training labels are 0=traversable and nonzero=non-traversable (rock).
        binary_mask = (rock_probability >= self.threshold).astype(np.uint8)
        class_confidence = np.where(
            binary_mask != 0, rock_probability, 1.0 - rock_probability
        )
        # Use class IDs 0 and 1 (rather than display intensities 0 and 255).
        mask_image = PILImage.fromarray(binary_mask, mode="L")
        confidence_image = PILImage.fromarray(
            np.clip(class_confidence * 255.0, 0, 255).astype(np.uint8), mode="L"
        )

        self.mask_publishers[camera].publish(
            self.make_image_message(message, mask_image, "mono8")
        )
        self.confidence_publishers[camera].publish(
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
