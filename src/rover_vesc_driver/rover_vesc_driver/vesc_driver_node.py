"""
rover_vesc_driver — cmd_vel to four VESC motor controllers over CAN.

Skid-steer mixing of geometry_msgs/Twist into per-wheel SET_RPM commands for
four VESCs on a shared CAN bus behind a CH340 USB-CAN adapter (python-can
'seeedstudio' interface). A single background thread owns the bus: every tick
it drains incoming VESC STATUS 1/2/4/5 frames into per-wheel telemetry, sends
wheel commands at the control rate and publishes BatteryState and per-wheel
diagnostics. WheelEncoders is published at the STATUS_5 telemetry rate — only
when every wheel has a new tachometer sample, stamped with the ROS time the
newest STATUS_5 frame was decoded, so consecutive messages carry genuinely new
counts and correct delta/dt at any broadcast rate (a valid=false keep-alive
still goes out at the control rate while telemetry is stale). ROS callbacks
only mutate shared command state under a lock.

Failsafes: cmd_vel deadman (coast on silence), per-wheel slew-rate limits,
erpm hard clamp, bus open/reopen retry with backoff, coast-all on shutdown.
"""
from __future__ import annotations

import math
import struct
import threading
import time
from dataclasses import dataclass
from typing import Optional

import can
import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import BatteryState

from rover_interfaces.msg import WheelEncoders

try:
    from serial.tools import list_ports
except Exception:  # pyserial should always be present (python3-serial dep)
    list_ports = None

# VESC CAN packet types (arbitration_id = (packet_type << 8) | controller_id)
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_CURRENT = 1
CAN_PACKET_SET_RPM = 3
CAN_PACKET_STATUS = 9         # erpm, motor current, duty
CAN_PACKET_STATUS_2 = 14      # amp-hours, amp-hours charged
CAN_PACKET_STATUS_4 = 16      # temp_fet, temp_motor, current_in
CAN_PACKET_STATUS_5 = 27      # tachometer, v_in

CH340_VID_PID = (0x1A86, 0x7523)          # CH340 USB-serial (seeed USB-CAN)
WHEEL_NAMES = ('vesc_fl', 'vesc_fr', 'vesc_rl', 'vesc_rr')
RX_DRAIN_PER_TICK = 32
SPEED_DEADBAND_MPS = 0.005
BATTERY_PERIOD_SEC = 0.5
DIAGNOSTICS_PERIOD_SEC = 1.0
CELL_EMPTY_V = 3.3
CELL_FULL_V = 4.15


@dataclass
class WheelTelemetry:
    """Latest decoded VESC status values for one controller (raw signs)."""
    erpm: float = 0.0
    duty: float = 0.0
    motor_current: float = 0.0
    input_current: float = 0.0
    temp_fet: float = 0.0
    temp_motor: float = 0.0
    v_in: float = 0.0
    tacho: int = 0
    amp_hours: float = 0.0
    t_status: float = 0.0
    t_status4: float = 0.0
    t_status5: float = 0.0
    stamp_status5: Optional[Time] = None  # ROS time of the newest STATUS_5


class VescDriverNode(Node):
    def __init__(self) -> None:
        super().__init__('vesc_driver_node')

        self.declare_parameter('can_interface', 'seeedstudio')
        self.declare_parameter('can_channel', '')     # '' -> autodetect CH340
        self.declare_parameter('can_bitrate', 500000)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('encoders_topic', '/wheel/encoders')
        self.declare_parameter('battery_topic', '/battery/state')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('control_rate_hz', 50.0)
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('feedback_timeout_sec', 0.35)
        self.declare_parameter('wheel_can_ids', [57, 25, 92, 71])       # FL FR RL RR
        self.declare_parameter('wheel_inverts', [True, True, False, False])
        self.declare_parameter('wheel_radius_m', 0.0825)
        self.declare_parameter('track_width_m', 0.40)
        self.declare_parameter('wheelbase_m', 0.50)   # unused in mixing; kept
        self.declare_parameter('gear_ratio', 3.0)
        self.declare_parameter('motor_pole_pairs', 7)
        self.declare_parameter('max_erpm', 4000)
        self.declare_parameter('min_erpm', 900)   # 0 = отключить подпор
        self.declare_parameter('max_wheel_speed_mps', 1.6)
        self.declare_parameter('max_wheel_accel_mps2', 1.5)
        self.declare_parameter('idle_mode', 'coast')  # 'coast' | 'hold'
        self.declare_parameter('battery_cells', 6)

        self.can_interface = str(self.get_parameter('can_interface').value)
        self.can_channel = str(self.get_parameter('can_channel').value)
        self.can_bitrate = int(self.get_parameter('can_bitrate').value)
        self.rate = float(self.get_parameter('control_rate_hz').value)
        if self.rate <= 0.0:
            raise ValueError('control_rate_hz must be positive')
        self.command_timeout = float(self.get_parameter('command_timeout_sec').value)
        self.feedback_timeout = float(self.get_parameter('feedback_timeout_sec').value)
        self.can_ids = [int(v) for v in self.get_parameter('wheel_can_ids').value]
        self.inverts = [bool(v) for v in self.get_parameter('wheel_inverts').value]
        if len(self.can_ids) != 4 or len(self.inverts) != 4:
            raise ValueError('wheel_can_ids and wheel_inverts must have four entries (FL FR RL RR)')
        self.signs = [-1 if inv else 1 for inv in self.inverts]
        self.wheel_radius = float(self.get_parameter('wheel_radius_m').value)
        self.track_width = float(self.get_parameter('track_width_m').value)
        self.wheelbase = float(self.get_parameter('wheelbase_m').value)
        self.gear_ratio = float(self.get_parameter('gear_ratio').value)
        self.pole_pairs = int(self.get_parameter('motor_pole_pairs').value)
        self.max_erpm = float(self.get_parameter('max_erpm').value)
        self.min_erpm = float(self.get_parameter('min_erpm').value)
        self.max_wheel_speed = float(self.get_parameter('max_wheel_speed_mps').value)
        self.max_accel = float(self.get_parameter('max_wheel_accel_mps2').value)
        self.idle_mode = str(self.get_parameter('idle_mode').value).lower()
        if self.idle_mode not in ('coast', 'hold'):
            self.get_logger().warning(f"Unknown idle_mode '{self.idle_mode}'; using 'coast'")
            self.idle_mode = 'coast'
        self.battery_cells = int(self.get_parameter('battery_cells').value)
        # erpm per m/s of wheel-rim speed
        self.erpm_per_mps = (
            self.gear_ratio * 60.0 * self.pole_pairs / (2.0 * math.pi * self.wheel_radius)
        )

        # shared command state (ROS callbacks write, CAN thread reads)
        self._lock = threading.Lock()
        self._target_mps = [0.0, 0.0, 0.0, 0.0]      # FL FR RL RR
        self._last_cmd: Optional[float] = None       # None -> no cmd_vel yet
        # CAN-thread-owned state
        self._tel: dict[int, WheelTelemetry] = {cid: WheelTelemetry() for cid in self.can_ids}
        self._pub_t_status5 = {cid: 0.0 for cid in self.can_ids}  # last published
        self._speed_mps = [0.0, 0.0, 0.0, 0.0]       # slew-limited output
        self._sequence = 0
        self._deadman = True                         # coast silently until first cmd
        self._last_battery = 0.0
        self._last_diag = 0.0
        self._log_state: dict[str, Optional[str]] = {}

        self.encoder_pub = self.create_publisher(
            WheelEncoders, str(self.get_parameter('encoders_topic').value), 20
        )
        self.battery_pub = self.create_publisher(
            BatteryState, str(self.get_parameter('battery_topic').value), 10
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter('diagnostics_topic').value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value), self._cmd, 10
        )

        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._can_loop, daemon=True)
        self._thread.start()

    # ---- ROS side ----------------------------------------------------------
    def _cmd(self, message: Twist) -> None:
        vx = float(message.linear.x)
        wz = float(message.angular.z)
        if not (math.isfinite(vx) and math.isfinite(wz)):
            self.get_logger().error('Ignored non-finite cmd_vel')
            return
        half_track = self.track_width / 2.0
        v_left = vx - wz * half_track
        v_right = vx + wz * half_track
        peak = max(abs(v_left), abs(v_right))
        if peak > self.max_wheel_speed:
            scale = self.max_wheel_speed / peak
            v_left *= scale
            v_right *= scale
        with self._lock:
            self._target_mps = [v_left, v_right, v_left, v_right]
            self._last_cmd = time.monotonic()

    # ---- CAN bus helpers ---------------------------------------------------
    def _log_once(self, key: str, message: Optional[str]) -> None:
        """Log an error only when it differs from the last one for this key."""
        if self._log_state.get(key) == message:
            return
        self._log_state[key] = message
        if message is not None:
            self.get_logger().error(message)

    def _detect_channel(self) -> Optional[str]:
        if list_ports is None:
            return None
        ports = list(list_ports.comports())
        matches = [p.device for p in ports
                   if p.vid == CH340_VID_PID[0] and p.pid == CH340_VID_PID[1]]
        if not matches:                   # fallback: any CH340 by description
            matches = [p.device for p in ports
                       if 'CH340' in (p.description or '')]
        if not matches:
            return None
        if len(matches) == 1:
            return matches[0]
        # Несколько одинаковых адаптеров: берём тот, где реально идёт
        # VESC-трафик (extended-id кадры), а не первый попавшийся.
        for device in matches:
            try:
                probe = can.Bus(interface=self.can_interface, channel=device,
                                bitrate=self.can_bitrate, timeout=0.1)
            except Exception:
                continue
            alive = False
            deadline = time.monotonic() + 1.5
            while time.monotonic() < deadline:
                try:
                    frame = probe.recv(timeout=0.2)
                except Exception:
                    break
                if frame is not None and frame.is_extended_id:
                    alive = True
                    break
            try:
                probe.shutdown()
            except Exception:
                pass
            if alive:
                self.get_logger().info(
                    f'{len(matches)} CH340 adapters found; {device} carries VESC traffic')
                return device
        self.get_logger().warning(
            f'{len(matches)} CH340 adapters found but none carries VESC traffic; '
            f'using {matches[0]} (проверьте питание VESC/шину)')
        return matches[0]

    def _open_bus(self) -> Optional[can.BusABC]:
        channel = self.can_channel or self._detect_channel()
        if not channel:
            self._log_once('bus', 'CH340 USB-CAN adapter not found; waiting for it')
            return None
        try:
            # Serial timeout must be generous: the seeedstudio driver assembles
            # each frame from multiple ser.read() calls that all use THIS
            # timeout (recv()'s own timeout arg is ignored). Too short -> mis-framing.
            bus = can.Bus(interface=self.can_interface, channel=channel,
                          bitrate=self.can_bitrate, timeout=0.1)
        except Exception as exc:
            self._log_once('bus', f'CAN open failed on {channel}: {exc!r}')
            return None
        self._log_once('bus', None)
        wheel_map = ' '.join(
            f'{name[-2:].upper()}=id{cid}{"(inv)" if inv else ""}'
            for name, cid, inv in zip(WHEEL_NAMES, self.can_ids, self.inverts)
        )
        self.get_logger().info(
            f'VESC CAN up: {self.can_interface}:{channel} @ {self.can_bitrate} bit/s; '
            f'wheels {wheel_map}'
        )
        return bus

    @staticmethod
    def _send_frame(bus: can.BusABC, ptype: int, cid: int, payload: bytes) -> None:
        bus.send(can.Message(arbitration_id=(ptype << 8) | cid,
                             is_extended_id=True, data=payload))

    def _send_current(self, bus: can.BusABC, cid: int, amps: float) -> None:
        self._send_frame(bus, CAN_PACKET_SET_CURRENT, cid,
                         struct.pack('>i', int(amps * 1000.0)))

    def _send_rpm(self, bus: can.BusABC, cid: int, erpm: float) -> None:
        self._send_frame(bus, CAN_PACKET_SET_RPM, cid, struct.pack('>i', int(erpm)))

    # ---- telemetry ---------------------------------------------------------
    def _decode_frame(self, frame: can.Message, now: float) -> None:
        if not frame.is_extended_id:
            return
        ptype = (frame.arbitration_id >> 8) & 0xFF
        cid = frame.arbitration_id & 0xFF
        tel = self._tel.get(cid)
        if tel is None:
            return
        d = frame.data
        try:
            if ptype == CAN_PACKET_STATUS and len(d) >= 8:
                tel.erpm = float(struct.unpack('>i', d[0:4])[0])
                tel.motor_current = struct.unpack('>h', d[4:6])[0] / 10.0
                tel.duty = struct.unpack('>h', d[6:8])[0] / 1000.0
                tel.t_status = now
            elif ptype == CAN_PACKET_STATUS_2 and len(d) >= 8:
                tel.amp_hours = struct.unpack('>i', d[0:4])[0] / 1e4
            elif ptype == CAN_PACKET_STATUS_4 and len(d) >= 6:
                tel.temp_fet = struct.unpack('>h', d[0:2])[0] / 10.0
                tel.temp_motor = struct.unpack('>h', d[2:4])[0] / 10.0
                tel.input_current = struct.unpack('>h', d[4:6])[0] / 10.0
                tel.t_status4 = now
            elif ptype == CAN_PACKET_STATUS_5 and len(d) >= 6:
                tel.tacho = int(struct.unpack('>i', d[0:4])[0])
                tel.v_in = struct.unpack('>h', d[4:6])[0] / 10.0
                tel.t_status5 = now
                tel.stamp_status5 = self.get_clock().now()
        except struct.error:
            return

    def _drain_rx(self, bus: can.BusABC, now: float) -> None:
        # recv(timeout=0) => driver does exactly one frame-assembly attempt
        # using the serial-port timeout. With a busy bus each returns a frame
        # almost instantly; break on the first miss so we don't stall.
        for _ in range(RX_DRAIN_PER_TICK):
            frame = bus.recv(timeout=0)
            if frame is None:
                break
            self._decode_frame(frame, now)

    # ---- command path ------------------------------------------------------
    def _send_commands(self, bus: can.BusABC, now: float, dt: float) -> None:
        with self._lock:
            targets = list(self._target_mps)
            last_cmd = self._last_cmd
        stale = last_cmd is None or (now - last_cmd) > self.command_timeout
        if stale != self._deadman:
            self._deadman = stale
            if stale:
                self.get_logger().warning('cmd_vel timeout: coasting all wheels')
            else:
                self.get_logger().info('cmd_vel stream active')
        if stale:
            # Deadman ALWAYS coasts, regardless of idle_mode.
            self._speed_mps = [0.0, 0.0, 0.0, 0.0]
            for cid in self.can_ids:
                self._send_current(bus, cid, 0.0)
            return
        for i, cid in enumerate(self.can_ids):
            current = self._speed_mps[i]
            target = targets[i]
            same_dir = current == 0.0 or target == 0.0 or current * target > 0.0
            speeding_up = same_dir and abs(target) > abs(current)
            limit = self.max_accel if speeding_up else 2.0 * self.max_accel
            step = limit * dt
            if abs(target - current) <= step:
                current = target
            else:
                current += math.copysign(step, target - current)
            self._speed_mps[i] = current
            if abs(current) < SPEED_DEADBAND_MPS:
                if self.idle_mode == 'hold':
                    self._send_rpm(bus, cid, 0.0)
                else:
                    self._send_current(bus, cid, 0.0)
                continue
            erpm = current * self.erpm_per_mps
            erpm = max(-self.max_erpm, min(self.max_erpm, erpm))
            # Ненастроенный sensorless FOC ниже min_erpm не держит обороты и
            # уходит в срыв/долбёжку (хаотичные рывки, импульсные токи) —
            # поднимаем команду до порога, сохраняя знак. min_erpm=0 отключает.
            if self.min_erpm > 0.0 and 0.0 < abs(erpm) < self.min_erpm:
                erpm = math.copysign(self.min_erpm, erpm)
            self._send_rpm(bus, cid, self.signs[i] * erpm)

    # ---- feedback publishing (runs in the CAN thread) ----------------------
    def _publish_feedback(self, now: float) -> None:
        stamp = self.get_clock().now().to_msg()
        self._publish_encoders(now, stamp)
        if now - self._last_battery >= BATTERY_PERIOD_SEC:
            self._last_battery = now
            self._publish_battery(now, stamp)
        if now - self._last_diag >= DIAGNOSTICS_PERIOD_SEC:
            self._last_diag = now
            self._publish_diagnostics(now, stamp)

    def _publish_encoders(self, now: float, stamp) -> None:
        counts: list[int] = []
        speeds: list[float] = []
        valid = True
        for i, cid in enumerate(self.can_ids):
            tel = self._tel[cid]
            fresh = (
                tel.t_status > 0.0 and tel.t_status5 > 0.0
                and (now - tel.t_status) <= self.feedback_timeout
                and (now - tel.t_status5) <= self.feedback_timeout
            )
            valid = valid and fresh
            counts.append(self.signs[i] * tel.tacho)
            speeds.append(self.signs[i] * tel.erpm / self.erpm_per_mps)
        if valid:
            # Publish only when every wheel got a new STATUS_5 since the last
            # message, stamped with the newest frame's decode time: consecutive
            # messages then carry genuinely new counts and their stamp spacing
            # matches the real telemetry interval, so downstream odometry sees
            # correct delta/dt at any status broadcast rate. While telemetry is
            # stale a valid=false keep-alive still goes out every tick below.
            if not all(
                self._tel[cid].t_status5 > self._pub_t_status5[cid]
                for cid in self.can_ids
            ):
                return
            newest = max(self.can_ids, key=lambda cid: self._tel[cid].t_status5)
            newest_stamp = self._tel[newest].stamp_status5
            if newest_stamp is not None:
                stamp = newest_stamp.to_msg()
            for cid in self.can_ids:
                self._pub_t_status5[cid] = self._tel[cid].t_status5
        message = WheelEncoders()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        message.total_counts = counts
        message.measured_mps = speeds
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        message.sequence = self._sequence
        message.valid = valid
        self.encoder_pub.publish(message)

    def _publish_battery(self, now: float, stamp) -> None:
        volts = [tel.v_in for tel in self._tel.values()
                 if tel.t_status5 > 0.0 and (now - tel.t_status5) <= self.feedback_timeout]
        amps = [tel.input_current for tel in self._tel.values()
                if tel.t_status4 > 0.0 and (now - tel.t_status4) <= self.feedback_timeout]
        message = BatteryState()
        message.header.stamp = stamp
        voltage = sum(volts) / len(volts) if volts else 0.0
        message.voltage = float(voltage)
        message.current = float(-sum(amps))   # discharge is negative per REP
        if volts and self.battery_cells > 0:
            fraction = (voltage / self.battery_cells - CELL_EMPTY_V) / (CELL_FULL_V - CELL_EMPTY_V)
            message.percentage = float(max(0.0, min(1.0, fraction)))
        message.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        message.present = bool(volts)
        self.battery_pub.publish(message)

    def _publish_diagnostics(self, now: float, stamp) -> None:
        array = DiagnosticArray()
        array.header.stamp = stamp
        for i, cid in enumerate(self.can_ids):
            tel = self._tel[cid]
            t_last = max(tel.t_status, tel.t_status4, tel.t_status5)
            fresh = t_last > 0.0 and (now - t_last) <= self.feedback_timeout
            status = DiagnosticStatus()
            status.name = WHEEL_NAMES[i]
            status.hardware_id = str(cid)
            status.level = DiagnosticStatus.OK if fresh else DiagnosticStatus.ERROR
            status.message = 'ok' if fresh else 'stale telemetry'
            sign = self.signs[i]
            status.values = [
                KeyValue(key='erpm', value=f'{sign * tel.erpm:.0f}'),
                KeyValue(key='duty', value=f'{tel.duty:.3f}'),
                KeyValue(key='motor_current', value=f'{tel.motor_current:.1f}'),
                KeyValue(key='input_current', value=f'{tel.input_current:.1f}'),
                KeyValue(key='temp_fet', value=f'{tel.temp_fet:.1f}'),
                KeyValue(key='temp_motor', value=f'{tel.temp_motor:.1f}'),
                KeyValue(key='v_in', value=f'{tel.v_in:.1f}'),
                KeyValue(key='tacho', value=str(sign * tel.tacho)),
            ]
            array.status.append(status)
        self.diag_pub.publish(array)

    # ---- the single CAN owner thread ---------------------------------------
    def _can_loop(self) -> None:
        bus: Optional[can.BusABC] = None
        backoff = 1.0
        period = 1.0 / self.rate
        last_tick = time.monotonic()
        while not self._stop.is_set():
            if bus is None:
                bus = self._open_bus()
                if bus is None:
                    # Bus down: keep publishing feedback while waiting out the
                    # backoff (encoders go valid=false, diagnostics ERROR) so
                    # the failure stays visible instead of the topics going silent.
                    deadline = time.monotonic() + backoff
                    while not self._stop.is_set():
                        try:
                            self._publish_feedback(time.monotonic())
                        except Exception:
                            pass
                        left = deadline - time.monotonic()
                        if left <= 0.0:
                            break
                        self._stop.wait(min(1.0, left))
                    backoff = min(backoff * 2.0, 10.0)
                    continue
                backoff = 1.0
                last_tick = time.monotonic()
            now = time.monotonic()
            dt = max(0.001, min(0.1, now - last_tick))
            last_tick = now
            try:
                self._drain_rx(bus, now)
                self._send_commands(bus, now, dt)
            except Exception as exc:
                self._log_once('bus', f'CAN I/O error: {exc!r}; reopening bus')
                try:
                    bus.shutdown()
                except Exception:
                    pass
                bus = None
                continue
            try:
                self._publish_feedback(now)
            except Exception:
                pass          # ROS context may be tearing down; keep CAN alive
            elapsed = time.monotonic() - now
            if elapsed < period:
                self._stop.wait(period - elapsed)
        if bus is not None:
            try:
                for cid in self.can_ids:    # safety: coast before closing
                    self._send_current(bus, cid, 0.0)
            except Exception:
                pass
            try:
                bus.shutdown()
            except Exception:
                pass

    def close(self) -> None:
        self._stop.set()
        self._thread.join(timeout=3.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[VescDriverNode] = None
    try:
        node = VescDriverNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
