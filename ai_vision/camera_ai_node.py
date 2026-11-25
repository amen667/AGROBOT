import cv2
import json
import math
import rclpy

from PIL import Image
from rclpy.node import Node
from std_msgs.msg import String
from plant_health_tflite import PlantClassifier


class CameraAINode(Node):

    def __init__(self):
        super().__init__("camera_ai_node")

        self.publisher = self.create_publisher(
            String,
            "/plant_health/result",
            10
        )

        self.classifier = PlantClassifier()

        self.get_logger().info("AI inspection node ready")

        self.timer = self.create_timer(5.0, self.inspect)

    def inspect(self):
        cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

        try:
            if not cap.isOpened():
                raise RuntimeError("Camera unavailable")

            for _ in range(10):
                ret, frame = cap.read()

            if not ret or frame is None:
                raise RuntimeError("Capture failed")

        except Exception as e:
            self.get_logger().error(str(e))
            return

        finally:
            cap.release()

        try:
            image = Image.fromarray(
                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            )

            results, elapsed, tomato_mass = self.classifier.classify(image)

            label, confidence = results[0]

            if not math.isfinite(confidence):
                raise RuntimeError("Invalid prediction")

            result = {
                "status": "inspection_complete",
                "prediction": label,
                "confidence": confidence,
                "tomato_probability": tomato_mass,
                "inference_ms": elapsed
            }

            msg = String()
            msg.data = json.dumps(result)

            self.publisher.publish(msg)
            self.get_logger().info(msg.data)

        except Exception as e:
            self.get_logger().error(str(e))


def main():
    rclpy.init()
    node = CameraAINode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
