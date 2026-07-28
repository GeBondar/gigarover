"""USB camera node: streams JPEG frames over a built-in WebSocket server.

Architecture: the camera is captured once and broadcast as JPEG binary
WebSocket messages. The web UI connects to the WebSocket directly, and the
`ws_image_publisher_node` republishes the same stream to the classic
`/image_raw` + `/image_raw/compressed` topics on demand. This node no longer
publishes image topics itself.

When the camera outputs MJPEG, frames are passed through without any
decode/re-encode (CAP_PROP_CONVERT_RGB = 0), which keeps CPU usage minimal.
"""

from __future__ import annotations

import json
import math
import threading
import time
from typing import Optional

import cv2
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rover_camera.ws_video import WebSocketVideoServer
from rover_interfaces.srv import GetFrame

JPEG_SOI = b'\xff\xd8'


def as_capture_source(device: str) -> str | int:
    text = device.strip()
    if text.isdigit():
        return int(text)
    return text


class UsbCameraNode(Node):
    def __init__(self) -> None:
        super().__init__('usb_camera_node')

        self.declare_parameter('device', '/dev/video0')
        self.declare_parameter('frame_id', 'camera_optical_frame')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('use_mjpeg', True)
        self.declare_parameter('jpeg_quality', 85)
        self.declare_parameter('reconnect_interval_sec', 2.0)
        self.declare_parameter('get_frame_service', 'get_frame')
        self.declare_parameter('ws_bind_address', '0.0.0.0')
        self.declare_parameter('ws_port', 8766)

        self.capture_lock = threading.RLock()
        self.frame_lock = threading.RLock()

        self.capture: Optional[cv2.VideoCapture] = None
        self.last_open_attempt = 0.0
        self.last_warn_time = 0.0
        self.reconfigure_capture = False
        self.shutdown_event = threading.Event()

        self.latest_jpeg: Optional[bytes] = None
        self.latest_frame_seq = 0
        self.latest_header_stamp = None
        self.latest_width = 0
        self.latest_height = 0
        self.latest_capture_monotonic = 0.0

        self.actual_width = 0
        self.actual_height = 0
        self.actual_fps = 0.0
        self.actual_fourcc = ''
        self.mjpeg_passthrough = False

        self.frames_captured = 0
        self.frames_streamed = 0
        self.read_failures = 0

        self._load_parameters()
        self._configure_services()
        self.add_on_set_parameters_callback(self._handle_parameter_update)

        self.ws_server: Optional[WebSocketVideoServer] = None
        self._start_ws_server()

        self.capture_thread = threading.Thread(
            target=self._capture_loop,
            name='usb-camera-capture',
            daemon=True,
        )
        self.capture_thread.start()

        self._request_reopen(force=True)

    def _load_parameters(self) -> None:
        self.device = str(self.get_parameter('device').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.use_mjpeg = bool(self.get_parameter('use_mjpeg').value)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.reconnect_interval = float(
            self.get_parameter('reconnect_interval_sec').value
        )
        self.get_frame_service_name = str(
            self.get_parameter('get_frame_service').value
        )
        self.ws_bind_address = str(self.get_parameter('ws_bind_address').value)
        self.ws_port = int(self.get_parameter('ws_port').value)
        self._validate_configuration()

    def _validate_configuration(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError('width and height must be positive')
        if not math.isfinite(self.fps) or self.fps <= 0.0:
            raise ValueError('fps must be finite and positive')
        if not 10 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be between 10 and 100')
        if (
            not math.isfinite(self.reconnect_interval)
            or self.reconnect_interval <= 0.0
        ):
            raise ValueError('reconnect_interval_sec must be finite and positive')
        if not self.get_frame_service_name.strip():
            raise ValueError('get_frame_service must not be empty')
        if not 1 <= self.ws_port <= 65535:
            raise ValueError('ws_port must be a valid TCP port')

    def _configure_services(self) -> None:
        if getattr(self, 'get_frame_service', None) is not None:
            self.destroy_service(self.get_frame_service)
        self.get_frame_service = self.create_service(
            GetFrame,
            self.get_frame_service_name,
            self._handle_get_frame,
        )

    def _start_ws_server(self) -> None:
        if self.ws_server is not None:
            self.ws_server.stop()
            self.ws_server = None
        server = WebSocketVideoServer(
            self.ws_bind_address,
            self.ws_port,
            on_client_connect=self._meta_json,
            log=lambda text: self.get_logger().info(text),
        )
        server.start()
        self.ws_server = server
        self.get_logger().info(
            f'Camera WebSocket stream on ws://{self.ws_bind_address}:{self.ws_port}'
        )

    def _meta_json(self) -> str:
        return json.dumps({
            'type': 'meta',
            'device': self.device,
            'frame_id': self.frame_id,
            'width': self.actual_width or self.width,
            'height': self.actual_height or self.height,
            'fps': self.actual_fps or self.fps,
            'fourcc': self.actual_fourcc,
            'mjpeg_passthrough': self.mjpeg_passthrough,
        })

    def _handle_parameter_update(
        self,
        parameters: list[Parameter],
    ) -> SetParametersResult:
        candidate = {
            'device': self.device,
            'frame_id': self.frame_id,
            'width': self.width,
            'height': self.height,
            'fps': self.fps,
            'use_mjpeg': self.use_mjpeg,
            'jpeg_quality': self.jpeg_quality,
            'reconnect_interval_sec': self.reconnect_interval,
            'get_frame_service': self.get_frame_service_name,
            'ws_bind_address': self.ws_bind_address,
            'ws_port': self.ws_port,
        }

        try:
            for parameter in parameters:
                if parameter.name not in candidate:
                    continue
                candidate[parameter.name] = parameter.value

            width = int(candidate['width'])
            height = int(candidate['height'])
            fps = float(candidate['fps'])
            jpeg_quality = int(candidate['jpeg_quality'])
            reconnect_interval = float(candidate['reconnect_interval_sec'])
            get_frame_service_name = str(candidate['get_frame_service'])
            ws_port = int(candidate['ws_port'])

            if width <= 0 or height <= 0:
                raise ValueError('width and height must be positive')
            if not math.isfinite(fps) or fps <= 0.0:
                raise ValueError('fps must be finite and positive')
            if not 10 <= jpeg_quality <= 100:
                raise ValueError('jpeg_quality must be between 10 and 100')
            if (
                not math.isfinite(reconnect_interval)
                or reconnect_interval <= 0.0
            ):
                raise ValueError(
                    'reconnect_interval_sec must be finite and positive'
                )
            if not get_frame_service_name.strip():
                raise ValueError('get_frame_service must not be empty')
            if not 1 <= ws_port <= 65535:
                raise ValueError('ws_port must be a valid TCP port')
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        old_capture = (
            self.device,
            self.width,
            self.height,
            self.fps,
            self.use_mjpeg,
            self.jpeg_quality,
        )
        old_service_name = self.get_frame_service_name
        old_ws = (self.ws_bind_address, self.ws_port)

        self.device = str(candidate['device'])
        self.frame_id = str(candidate['frame_id'])
        self.width = width
        self.height = height
        self.fps = fps
        self.use_mjpeg = bool(candidate['use_mjpeg'])
        self.jpeg_quality = jpeg_quality
        self.reconnect_interval = reconnect_interval
        self.get_frame_service_name = get_frame_service_name
        self.ws_bind_address = str(candidate['ws_bind_address'])
        self.ws_port = ws_port

        if old_capture != (
            self.device,
            self.width,
            self.height,
            self.fps,
            self.use_mjpeg,
            self.jpeg_quality,
        ):
            self._request_reopen(force=True)

        if old_service_name != self.get_frame_service_name:
            self._configure_services()

        if old_ws != (self.ws_bind_address, self.ws_port):
            try:
                self._start_ws_server()
            except OSError as exc:
                return SetParametersResult(
                    successful=False,
                    reason=f'Cannot bind WebSocket server: {exc}',
                )

        self.get_logger().info(
            'Camera parameters updated: '
            f'{self.width}x{self.height} @ {self.fps:.1f} fps, '
            f'MJPEG={self.use_mjpeg}, ws_port={self.ws_port}'
        )
        return SetParametersResult(successful=True)

    def _warn_throttled(self, text: str) -> None:
        now = time.monotonic()
        if now - self.last_warn_time >= 2.0:
            self.get_logger().warning(text)
            self.last_warn_time = now

    def _request_reopen(self, *, force: bool = False) -> None:
        with self.capture_lock:
            self.reconfigure_capture = True
            if force:
                self.last_open_attempt = 0.0
            if self.capture is not None:
                self.capture.release()
                self.capture = None

    def _open_camera(self) -> bool:
        now = time.monotonic()
        if now - self.last_open_attempt < self.reconnect_interval:
            return False

        self.last_open_attempt = now
        source = as_capture_source(self.device)
        backends = [cv2.CAP_V4L2, cv2.CAP_ANY]

        for backend in backends:
            capture = cv2.VideoCapture(source, backend)
            if not capture.isOpened():
                capture.release()
                continue

            capture.set(cv2.CAP_PROP_BUFFERSIZE, 1.0)
            if self.use_mjpeg:
                capture.set(
                    cv2.CAP_PROP_FOURCC,
                    float(cv2.VideoWriter_fourcc(*'MJPG')),
                )
            capture.set(cv2.CAP_PROP_FRAME_WIDTH, float(self.width))
            capture.set(cv2.CAP_PROP_FRAME_HEIGHT, float(self.height))
            capture.set(cv2.CAP_PROP_FPS, float(self.fps))

            fourcc = int(capture.get(cv2.CAP_PROP_FOURCC))
            fourcc_text = ''.join(
                chr((fourcc >> shift) & 0xFF) for shift in (0, 8, 16, 24)
            ).strip('\x00')

            # If the camera really delivers MJPEG, take the encoded bytes
            # as-is and skip the decode + re-encode round-trip entirely.
            passthrough = False
            if self.use_mjpeg and fourcc_text.upper() == 'MJPG':
                passthrough = bool(capture.set(cv2.CAP_PROP_CONVERT_RGB, 0.0))

            with self.capture_lock:
                if self.capture is not None:
                    self.capture.release()
                self.capture = capture
                self.reconfigure_capture = False

            self.actual_width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
            self.actual_height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
            self.actual_fps = float(capture.get(cv2.CAP_PROP_FPS))
            self.actual_fourcc = fourcc_text
            self.mjpeg_passthrough = passthrough
            self.get_logger().info(
                f'USB camera connected: {self.device} -> '
                f'{self.actual_width}x{self.actual_height} @ '
                f'{self.actual_fps:.1f} fps, fourcc={self.actual_fourcc or "unknown"}, '
                f'passthrough={"on" if passthrough else "off"}'
            )
            if self.ws_server is not None:
                self.ws_server.broadcast_text(self._meta_json())
            return True

        self._warn_throttled(
            f'Cannot open USB camera {self.device}; retrying automatically'
        )
        return False

    def _frame_to_jpeg(self, frame: np.ndarray) -> Optional[bytes]:
        if self.mjpeg_passthrough:
            data = frame.tobytes()
            if data[:2] == JPEG_SOI:
                return data
            # The driver handed us something that is not JPEG after all;
            # fall back to the decode + encode path on the next reopen.
            self._warn_throttled(
                'MJPEG passthrough returned non-JPEG data; disabling passthrough'
            )
            self.mjpeg_passthrough = False
            self._request_reopen(force=True)
            return None

        if len(frame.shape) != 3 or frame.shape[2] != 3:
            self._warn_throttled('Camera returned a non-BGR frame; skipping')
            return None
        ok, encoded = cv2.imencode(
            '.jpg',
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), self.jpeg_quality],
        )
        if not ok:
            self._warn_throttled('OpenCV could not encode frame as JPEG')
            return None
        return encoded.tobytes()

    def _capture_loop(self) -> None:
        while not self.shutdown_event.is_set():
            with self.capture_lock:
                capture = self.capture
                needs_reopen = self.reconfigure_capture

            if needs_reopen or capture is None or not capture.isOpened():
                if not self._open_camera():
                    self.shutdown_event.wait(0.05)
                continue

            ok, frame = capture.read()
            if not ok or frame is None:
                self.read_failures += 1
                self._warn_throttled(
                    f'Failed to read frame from {self.device}; reconnecting'
                )
                self._request_reopen()
                self.shutdown_event.wait(0.05)
                continue

            jpeg = self._frame_to_jpeg(frame)
            if jpeg is None:
                self.shutdown_event.wait(0.001)
                continue

            with self.frame_lock:
                self.latest_jpeg = jpeg
                self.latest_frame_seq += 1
                self.latest_header_stamp = self.get_clock().now().to_msg()
                self.latest_width = self.actual_width or self.width
                self.latest_height = self.actual_height or self.height
                self.latest_capture_monotonic = time.monotonic()
                self.frames_captured += 1

            if self.ws_server is not None and self.ws_server.client_count() > 0:
                self.ws_server.broadcast_frame(jpeg)
                self.frames_streamed += 1

        self._request_reopen(force=True)

    def _handle_get_frame(
        self,
        _request: GetFrame.Request,
        response: GetFrame.Response,
    ) -> GetFrame.Response:
        with self.frame_lock:
            latest_jpeg = self.latest_jpeg
            stamp = self.latest_header_stamp
            width = self.latest_width
            height = self.latest_height
            age_sec = (
                max(0.0, time.monotonic() - self.latest_capture_monotonic)
                if self.latest_capture_monotonic > 0.0
                else float('inf')
            )

        if latest_jpeg is None or stamp is None or width <= 0 or height <= 0:
            response.success = False
            response.message = 'No camera frame is available yet'
            response.age_sec = float('inf')
            return response

        response.success = True
        response.message = 'ok'
        response.frame.header.stamp = stamp
        response.frame.header.frame_id = self.frame_id
        response.frame.format = 'jpeg'
        response.frame.data = latest_jpeg
        response.width = int(width)
        response.height = int(height)
        response.age_sec = float(age_sec)
        return response

    def close(self) -> None:
        self.shutdown_event.set()
        self._request_reopen(force=True)
        if self.ws_server is not None:
            self.ws_server.stop()
        if hasattr(self, 'capture_thread') and self.capture_thread.is_alive():
            self.capture_thread.join(timeout=1.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[UsbCameraNode] = None
    try:
        node = UsbCameraNode()
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
