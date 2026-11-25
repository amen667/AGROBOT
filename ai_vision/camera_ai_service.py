import cv2
import json
import math
import rclpy

from PIL import Image
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from plant_health_tflite import PlantClassifier


class CameraAIService(Node):

    def __init__(self):
        super().__init__("camera_ai_service")

        self.classifier = PlantClassifier()

        self.publisher = self.create_publisher(
            String, "/plant_health/result", 10
        )

        self.service = self.create_service(
            Trigger,
            "/plant_health/inspect",
            self.inspect
        )

        self.get_logger().info(
            "AI service ready. Waiting for inspection request."
        )

    def inspect(self, request, response):

        self.get_logger().info("Inspection requested")

        try:
            cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

            try:
                if not cap.isOpened():
                    raise RuntimeError("Camera unavailable")

                for _ in range(10):
                    ret, frame = cap.read()

                if not ret or frame is None:
                    raise RuntimeError("Capture failed")
            finally:
                cap.release()

            image = Image.fromarray(
                cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            )

            results, elapsed, tomato_mass = \
                self.classifier.classify(image)

            label, confidence = results[0]

            if not math.isfinite(confidence):
                raise RuntimeError("Invalid AI result")

            result = {
                "status": "inspection_complete",
                "prediction": label,
                "confidence": confidence,
                "inference_ms": elapsed
            }

            msg = String()
            msg.data = json.dumps(result)

            self.publisher.publish(msg)

            response.success = True
            response.message = msg.data

            self.get_logger().info(msg.data)

        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(str(e))

        return response


def main():
    rclpy.init()
    node = CameraAIService()

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
