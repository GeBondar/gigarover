"""Republishes the camera WebSocket stream to classic ROS image topics.

Connects to the camera node's WebSocket (JPEG frames) and publishes
`/image_raw/compressed` (byte passthrough) and `/image_raw` (decoded BGR).
Completely lazy: while no node subscribes to either topic, the WebSocket
stays disconnected and no CPU is spent. Raw frames are decoded only while
someone actually listens to the raw topic.
"""

from __future__ import annotations

import json
import threading
import time
from typing import Optional

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from rover_camera.ws_video import OP_BINARY, OP_TEXT, WebSocketVideoClient
from sensor_msgs.msg import CompressedImage, Image


class WsImagePublisherNode(Node):
    def __init__(self) -> None:
        super().__init__('ws_image_publisher_node')

        self.declare_parameter('ws_url', 'ws://127.0.0.1:8766')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('compressed_image_topic', '/image_raw/compressed')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('publish_raw', True)
        self.declare_parameter('publish_compressed', True)
        self.declare_parameter('reconnect_interval_sec', 2.0)

        self._load_parameters()

        self.shutdown_event = threading.Event()
        self.last_warn_time = 0.0
        self.frames_received = 0
        self.frames_published_raw = 0
        self.frames_published_compressed = 0
        self.stream_meta: dict = {}

        self.raw_publisher = None
        self.compressed_publisher = None
        self._configure_publishers()
        self.add_on_set_parameters_callback(self._handle_parameter_update)

        self.worker = threading.Thread(
            target=self._stream_loop,
            name='ws-image-republisher',
            daemon=True,
        )
        self.worker.start()

    def _load_parameters(self) -> None:
        self.ws_url = str(self.get_parameter('ws_url').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.compressed_image_topic = str(
            self.get_parameter('compressed_image_topic').value
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.publish_raw = bool(self.get_parameter('publish_raw').value)
        self.publish_compressed = bool(
            self.get_parameter('publish_compressed').value
        )
        self.reconnect_interval = max(
            0.2,
            float(self.get_parameter('reconnect_interval_sec').value),
        )

    def _configure_publishers(self) -> None:
        if self.raw_publisher is not None:
            self.destroy_publisher(self.raw_publisher)
            self.raw_publisher = None
        if self.compressed_publisher is not None:
            self.destroy_publisher(self.compressed_publisher)
            self.compressed_publisher = None

        if self.publish_raw:
            self.raw_publisher = self.create_publisher(
                Image,
                self.image_topic,
                qos_profile_sensor_data,
            )
        if self.publish_compressed:
            self.compressed_publisher = self.create_publisher(
                CompressedImage,
                self.compressed_image_topic,
                qos_profile_sensor_data,
            )

    def _handle_parameter_update(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        try:
            for parameter in parameters:
                if parameter.name == 'reconnect_interval_sec':
                    if float(parameter.value) <= 0.0:
                        raise ValueError(
                            'reconnect_interval_sec must be positive'
                        )
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        names = {parameter.name for parameter in parameters}

        def apply() -> None:
            old_topics = (
                self.image_topic,
                self.compressed_image_topic,
                self.publish_raw,
                self.publish_compressed,
            )
            self._load_parameters()
            if old_topics != (
                self.image_topic,
                self.compressed_image_topic,
                self.publish_raw,
                self.publish_compressed,
            ):
                self._configure_publishers()

        # Defer briefly so the new values are visible via get_parameter().
        if names:
            threading.Timer(0.1, apply).start()
        return SetParametersResult(successful=True)

    def _warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_time >= 2.0:
            self.get_logger().warning(text)
            self.last_warn_time = now

    def _subscriber_demand(self) -> tuple[bool, bool]:
        raw_wanted = (
            self.raw_publisher is not None
            and self.raw_publisher.get_subscription_count() > 0
        )
        compressed_wanted = (
            self.compressed_publisher is not None
            and self.compressed_publisher.get_subscription_count() > 0
        )
        return raw_wanted, compressed_wanted

    def _stream_loop(self) -> None:
        client: Optional[WebSocketVideoClient] = None
        connected_url = ''
        while not self.shutdown_event.is_set():
            raw_wanted, compressed_wanted = self._subscriber_demand()
            if not raw_wanted and not compressed_wanted:
                if client is not None:
                    client.close()
                    client = None
                    self.get_logger().info(
                        'No image topic subscribers; camera WebSocket released'
                    )
                self.shutdown_event.wait(0.5)
                continue

            if client is not None and connected_url != self.ws_url:
                client.close()
                client = None

            if client is None:
                try:
                    client = WebSocketVideoClient(self.ws_url)
                    client.connect()
                    connected_url = self.ws_url
                    self.get_logger().info(
                        f'Connected to camera stream {self.ws_url}'
                    )
                except (OSError, ConnectionError, ValueError) as exc:
                    client = None
                    self._warn_throttled(
                        f'Camera WebSocket unavailable ({self.ws_url}): {exc}'
                    )
                    self.shutdown_event.wait(self.reconnect_interval)
                    continue

            try:
                opcode, payload = client.receive(timeout=2.0)
            except TimeoutError:
                continue
            except (OSError, ConnectionError) as exc:
                self._warn_throttled(f'Camera stream interrupted: {exc}')
                client.close()
                client = None
                self.shutdown_event.wait(self.reconnect_interval)
                continue

            if opcode == OP_TEXT:
                try:
                    meta = json.loads(payload.decode('utf-8'))
                    if isinstance(meta, dict):
                        self.stream_meta = meta
                except (ValueError, UnicodeDecodeError):
                    pass
                continue
            if opcode != OP_BINARY or not payload:
                continue

            self.frames_received += 1
            stamp = self.get_clock().now().to_msg()

            if compressed_wanted and self.compressed_publisher is not None:
                message = CompressedImage()
                message.header.stamp = stamp
                message.header.frame_id = self.frame_id
                message.format = 'jpeg'
                message.data = payload
                self.compressed_publisher.publish(message)
                self.frames_published_compressed += 1

            if raw_wanted and self.raw_publisher is not None:
                frame = cv2.imdecode(
                    np.frombuffer(payload, dtype=np.uint8),
                    cv2.IMREAD_COLOR,
                )
                if frame is None:
                    self._warn_throttled('Received frame is not decodable JPEG')
                    continue
                message = Image()
                message.header.stamp = stamp
                message.header.frame_id = self.frame_id
                message.height = int(frame.shape[0])
                message.width = int(frame.shape[1])
                message.encoding = 'bgr8'
                message.is_bigendian = False
                message.step = int(frame.shape[1] * 3)
                message.data = frame.tobytes()
                self.raw_publisher.publish(message)
                self.frames_published_raw += 1

        if client is not None:
            client.close()

    def close(self) -> None:
        self.shutdown_event.set()
        if self.worker.is_alive():
            self.worker.join(timeout=2.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[WsImagePublisherNode] = None
    try:
        node = WsImagePublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
