#!/usr/bin/env python3
"""Single-owner USB camera, live preview and inspection service for AGROBOT."""
import json
import math
import time
import uuid

import cv2
from PIL import Image
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String
from std_srvs.srv import Trigger
from plant_health_tflite import PlantClassifier


class LiveCameraService(Node):
    def __init__(self):
        super().__init__('camera_ai_live_service')
        self.classifier = PlantClassifier()
        self.cap = None
        self.ready = False
        self.warmup = 0
        self.retry_at = 0.0
        self.session = uuid.uuid4().hex[:12]
        self.count = 0
        self.preview = self.create_publisher(CompressedImage, '/plant_health/camera/preview', qos_profile_sensor_data)
        self.snapshot = self.create_publisher(CompressedImage, '/plant_health/camera/snapshot', 10)
        self.events = self.create_publisher(String, '/plant_health/camera/event', 20)
        self.results = self.create_publisher(String, '/plant_health/result', 10)
        self.service = self.create_service(Trigger, '/plant_health/inspect', self.inspect)
        self.timer = self.create_timer(0.15, self.preview_tick)
        self.get_logger().info('Live camera AI inspection service started')

    def event(self, phase, ident=None, detail=''):
        data = {'phase': phase, 'inspection_id': ident, 'detail': detail, 'timestamp': time.time()}
        msg = String(); msg.data = json.dumps(data)
        self.events.publish(msg)
        self.get_logger().info(f'{phase}: {detail}')

    def close_camera(self):
        if self.cap is not None:
            self.cap.release()
        self.cap = None
        self.ready = False
        self.warmup = 0

    def open_camera(self):
        if self.cap is not None:
            return True
        if time.monotonic() < self.retry_at:
            return False
        self.retry_at = time.monotonic() + 2.0
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.close_camera()
            self.event('camera_error', detail='Unable to open USB camera index 0')
            return False
        # Driver may ignore this hint; no dependency on it.
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        self.event('camera_warming', detail='Warming up camera')
        return True

    def read_frame(self):
        ok, frame = self.cap.read()
        if not ok or frame is None or not frame.size:
            self.close_camera()
            self.retry_at = time.monotonic() + 2.0
            raise RuntimeError('USB camera frame unavailable')
        return frame

    def publish_image(self, publisher, frame, ident, preview=False):
        # Preview downscaled to limit traffic. Inspection image retains original dimensions.
        if preview and frame.shape[1] > 480:
            height = max(1, round(frame.shape[0] * 480 / frame.shape[1]))
            frame = cv2.resize(frame, (480, height), interpolation=cv2.INTER_AREA)
        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 72 if preview else 95])
        if not ok:
            raise RuntimeError('JPEG encoding failed')
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = ident
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        publisher.publish(msg)

    def preview_tick(self):
        try:
            if not self.open_camera():
                return
            frame = self.read_frame()
            if not self.ready:
                self.warmup += 1
                if self.warmup < 10:
                    return
                self.ready = True
                self.event('camera_ready', detail='Live preview available')
            self.publish_image(self.preview, frame, 'camera', preview=True)
        except Exception as exc:
            self.close_camera()
            self.retry_at = time.monotonic() + 2.0
            self.event('camera_error', detail=str(exc))

    def inspect(self, request, response):
        self.count += 1
        ident = f'{self.session}-{self.count}'
        try:
            if not self.ready or self.cap is None:
                raise RuntimeError('Camera not ready; wait for live preview')
            self.event('capturing', ident, 'Capturing inspection frame')
            frame = self.read_frame()
            # Both the displayed snapshot and inference originate from this same frame.
            self.publish_image(self.snapshot, frame, ident)
            self.event('analyzing', ident, 'Frame captured; running TFLite inference')
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
            ranked, elapsed, tomato_mass = self.classifier.classify(image)
            label, confidence = ranked[0]
            confidence, elapsed, tomato_mass = float(confidence), float(elapsed), float(tomato_mass)
            if not all(math.isfinite(v) for v in (confidence, elapsed, tomato_mass)):
                raise ValueError('Non-finite AI output')
            result = {'status': 'inspection_complete', 'prediction': str(label),
                      'confidence': confidence, 'inference_ms': elapsed,
                      'tomato_probability': tomato_mass, 'inspection_id': ident}
            payload = json.dumps(result, allow_nan=False)
            msg = String(); msg.data = payload
            self.results.publish(msg)
            self.event('complete', ident, f'{label} | {elapsed:.1f} ms')
            response.success = True
            response.message = payload
        except Exception as exc:
            self.event('inspection_error', ident, str(exc))
            response.success = False
            response.message = str(exc)
        return response


def main():
    rclpy.init()
    node = None
    try:
        node = LiveCameraService()
        rclpy.spin(node)  # Single-threaded: preview and inspection never open/read camera concurrently.
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close_camera()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
