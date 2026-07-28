"""vesc_bridge_node — тонкий ROS-мост к демону rover-motord.

CAN-адаптером владеет rover-motord (rover_ws/tools/motord, systemd-юнит
rover-motord). Этот узел не трогает шину вообще:

  * /cmd_vel (QoS keep_last(1) + best_effort: всегда только самая свежая
    команда, без реплея очереди после стопора) -> UDP-датаграмма
    {"src":"ros","cmd":"drive"} демону. Deadman — на стороне демона,
    поэтому рестарт ROS-стека безопасен: моторы коастятся через 0.5 с.
  * Поток state от демона (подписка обновляется раз в секунду) ->
    WheelEncoders / BatteryState / DiagnosticArray. Семантика энкодеров
    прежнего драйвера сохранена: сообщение публикуется только когда все
    четыре колеса дали новый STATUS_5 (enc.seq вырос), штамп = now - age
    телеметрии, а при устаревших данных на частоте publish_rate идёт
    keep-alive с valid=false. Одометрия downstream работает без изменений.
  * Диагностика: прежние per-wheel статусы vesc_fl..vesc_rr плюс статус
    линка vesc_can_link с метриками демона (state линка, rx Гц, ошибки
    декодирования, переоткрытия шины). Если демон недоступен — ERROR.

Прямой доступ к CAN остался в vesc_driver_node (base_driver.type:
vesc_direct) как режим отката — перед его использованием остановите
rover-motord.
"""
from __future__ import annotations

import json
import math
import socket
import threading
import time
from typing import Any, Optional

import rclpy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import Twist
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import BatteryState

from rover_interfaces.msg import WheelEncoders

WHEEL_NAMES = ('vesc_fl', 'vesc_fr', 'vesc_rl', 'vesc_rr')
BATTERY_PERIOD_SEC = 0.5
DIAGNOSTICS_PERIOD_SEC = 1.0
SUB_PERIOD_SEC = 1.0
MAX_STAMP_AGE_SEC = 0.5           # страховка от кривого age из датаграммы


class VescBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('vesc_bridge_node')

        self.declare_parameter('motord_host', '127.0.0.1')
        self.declare_parameter('motord_port', 8460)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('encoders_topic', '/wheel/encoders')
        self.declare_parameter('battery_topic', '/battery/state')
        self.declare_parameter('diagnostics_topic', '/diagnostics')
        self.declare_parameter('publish_rate_hz', 50.0)
        self.declare_parameter('state_timeout_sec', 1.0)

        host = str(self.get_parameter('motord_host').value)
        port = int(self.get_parameter('motord_port').value)
        self._addr = (host, port)
        rate = float(self.get_parameter('publish_rate_hz').value)
        if rate <= 0.0:
            raise ValueError('publish_rate_hz must be positive')
        self._state_timeout = float(self.get_parameter('state_timeout_sec').value)

        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.bind(('127.0.0.1', 0))
        self._sock.settimeout(0.5)

        # последний state демона (пишет UDP-поток, читают таймеры executor'а)
        self._lock = threading.Lock()
        self._state: Optional[dict] = None
        self._state_t = 0.0               # monotonic прихода
        self._last_enc_seq = -1
        self._sequence = 0
        self._reachable: Optional[bool] = None

        self.encoder_pub = self.create_publisher(
            WheelEncoders, str(self.get_parameter('encoders_topic').value), 20
        )
        self.battery_pub = self.create_publisher(
            BatteryState, str(self.get_parameter('battery_topic').value), 10
        )
        self.diag_pub = self.create_publisher(
            DiagnosticArray, str(self.get_parameter('diagnostics_topic').value), 10
        )
        # Только самая свежая команда: после любого стопора DDS не должен
        # реплеить очередь устаревших cmd_vel в демона.
        cmd_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            Twist, str(self.get_parameter('cmd_vel_topic').value),
            self._cmd, cmd_qos,
        )

        self._stop = threading.Event()
        self._rx_thread = threading.Thread(
            target=self._state_loop, name='motord_rx', daemon=True
        )
        self._rx_thread.start()

        self._send_json({'v': 1, 'src': 'ros', 'cmd': 'sub'})
        self.create_timer(SUB_PERIOD_SEC, self._refresh_sub)
        self.create_timer(1.0 / rate, self._tick_encoders)
        self.create_timer(BATTERY_PERIOD_SEC, self._tick_battery)
        self.create_timer(DIAGNOSTICS_PERIOD_SEC, self._tick_diagnostics)

        self.get_logger().info(
            f'VESC bridge -> rover-motord {host}:{port} '
            '(CAN-адаптером владеет демон)'
        )

    # ---- UDP ----------------------------------------------------------------
    def _send_json(self, payload: dict) -> None:
        try:
            self._sock.sendto(
                json.dumps(payload, separators=(',', ':')).encode('utf-8'),
                self._addr,
            )
        except OSError:
            pass                          # демон недоступен — увидим по state

    def _state_loop(self) -> None:
        while not self._stop.is_set():
            try:
                data, _ = self._sock.recvfrom(65535)
            except socket.timeout:
                continue
            except ConnectionResetError:
                continue                  # ICMP unreachable: демон ещё не поднялся
            except OSError:
                if self._stop.is_set():
                    break
                time.sleep(0.1)
                continue
            try:
                msg = json.loads(data.decode('utf-8'))
            except ValueError:
                continue
            if not isinstance(msg, dict) or msg.get('type') != 'state':
                continue
            with self._lock:
                self._state = msg
                self._state_t = time.monotonic()

    def _refresh_sub(self) -> None:
        self._send_json({'v': 1, 'src': 'ros', 'cmd': 'sub'})

    # ---- команды ------------------------------------------------------------
    def _cmd(self, message: Twist) -> None:
        vx = float(message.linear.x)
        wz = float(message.angular.z)
        if not (math.isfinite(vx) and math.isfinite(wz)):
            self.get_logger().error('Ignored non-finite cmd_vel')
            return
        self._send_json({'v': 1, 'src': 'ros', 'cmd': 'drive',
                         'vx': vx, 'wz': wz})

    # ---- состояние ----------------------------------------------------------
    def _current_state(self) -> tuple[Optional[dict], bool]:
        """(state, fresh): fresh=False — демон молчит дольше state_timeout."""
        with self._lock:
            state = self._state
            age = time.monotonic() - self._state_t
        fresh = state is not None and age <= self._state_timeout
        if fresh != self._reachable:
            self._reachable = fresh
            if fresh:
                self.get_logger().info('rover-motord на связи')
            else:
                self.get_logger().error(
                    'rover-motord недоступен (нет state-датаграмм) — '
                    'энкодеры valid=false; проверьте службу rover-motord'
                )
        return state, fresh

    # ---- публикации ---------------------------------------------------------
    def _publish_encoders(self, counts, mps, valid: bool, stamp) -> None:
        message = WheelEncoders()
        message.header.stamp = stamp
        message.header.frame_id = 'base_link'
        message.total_counts = [int(v) for v in counts]
        message.measured_mps = [float(v) for v in mps]
        self._sequence = (self._sequence + 1) & 0xFFFFFFFF
        message.sequence = self._sequence
        message.valid = valid
        self.encoder_pub.publish(message)

    def _tick_encoders(self) -> None:
        state, fresh = self._current_state()
        now = self.get_clock().now()
        if not fresh:
            self._publish_encoders([0] * 4, [0.0] * 4, False, now.to_msg())
            return
        enc = state.get('enc') or {}
        counts = enc.get('counts') or [0] * 4
        mps = enc.get('mps') or [0.0] * 4
        seq = int(enc.get('seq') or 0)
        valid = bool(enc.get('valid'))
        if not valid:
            # Телеметрия устарела: keep-alive на частоте publish_rate, как в
            # старом драйвере, чтобы downstream видел отказ, а не тишину.
            self._publish_encoders(counts, mps, False, now.to_msg())
            return
        if seq < self._last_enc_seq:
            self._last_enc_seq = -1       # демон перезапустился, seq обнулился
        if seq == self._last_enc_seq:
            return                        # нового полного сэмпла ещё нет
        self._last_enc_seq = seq
        age_ms = enc.get('age_ms')
        age = min(max(float(age_ms) / 1000.0, 0.0), MAX_STAMP_AGE_SEC) \
            if age_ms is not None else 0.0
        stamp = (now - Duration(seconds=age)).to_msg()
        self._publish_encoders(counts, mps, True, stamp)

    def _tick_battery(self) -> None:
        state, fresh = self._current_state()
        message = BatteryState()
        message.header.stamp = self.get_clock().now().to_msg()
        battery = (state.get('battery') or {}) if (fresh and state) else {}
        voltage = float(battery.get('voltage') or 0.0)
        message.voltage = voltage
        message.current = -float(battery.get('input_current') or 0.0)
        percentage = battery.get('percentage')
        if percentage is not None:
            message.percentage = float(percentage)
        message.power_supply_technology = \
            BatteryState.POWER_SUPPLY_TECHNOLOGY_LION
        message.present = bool(battery.get('present')) if fresh else False
        self.battery_pub.publish(message)

    def _tick_diagnostics(self) -> None:
        state, fresh = self._current_state()
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        wheels = (state.get('wheels') or []) if (fresh and state) else []
        for i, name in enumerate(WHEEL_NAMES):
            wheel = wheels[i] if i < len(wheels) else {}
            status = DiagnosticStatus()
            status.name = name
            status.hardware_id = str(wheel.get('can_id', ''))
            wheel_fresh = bool(wheel.get('fresh')) if fresh else False
            status.level = (DiagnosticStatus.OK if wheel_fresh
                            else DiagnosticStatus.ERROR)
            status.message = ('ok' if wheel_fresh else
                              'stale telemetry' if fresh else
                              'motord unreachable')
            status.values = [
                KeyValue(key='erpm', value=f"{wheel.get('erpm', 0.0):.0f}"),
                KeyValue(key='duty', value=f"{wheel.get('duty', 0.0):.3f}"),
                KeyValue(key='motor_current',
                         value=f"{wheel.get('motor_current', 0.0):.1f}"),
                KeyValue(key='input_current',
                         value=f"{wheel.get('input_current', 0.0):.1f}"),
                KeyValue(key='temp_fet',
                         value=f"{wheel.get('temp_fet', 0.0):.1f}"),
                KeyValue(key='temp_motor',
                         value=f"{wheel.get('temp_motor', 0.0):.1f}"),
                KeyValue(key='v_in', value=f"{wheel.get('v_in', 0.0):.1f}"),
                KeyValue(key='tacho', value=str(wheel.get('tacho', 0))),
                KeyValue(key='status1_hz',
                         value=str(wheel.get('status1_hz', 0.0))),
                KeyValue(key='status5_hz',
                         value=str(wheel.get('status5_hz', 0.0))),
                KeyValue(key='max_gap_ms',
                         value=str(wheel.get('max_gap_ms', 0))),
            ]
            array.status.append(status)
        link = DiagnosticStatus()
        link.name = 'vesc_can_link'
        link.hardware_id = f'{self._addr[0]}:{self._addr[1]}'
        if not fresh:
            link.level = DiagnosticStatus.ERROR
            link.message = 'motord unreachable'
        else:
            link_state = str((state.get('link') or {}).get('state', 'down'))
            drive = state.get('drive') or {}
            can_info = state.get('can') or {}
            link.level = (DiagnosticStatus.OK if link_state == 'ok'
                          else DiagnosticStatus.WARN if link_state == 'degraded'
                          else DiagnosticStatus.ERROR)
            link.message = link_state
            link_metrics = state.get('link') or {}
            link.values = [
                KeyValue(key='rx_hz', value=str(link_metrics.get('rx_hz', 0))),
                KeyValue(key='max_gap_ms',
                         value=str(link_metrics.get('max_gap_ms', 0))),
                KeyValue(key='gaps_over_150ms',
                         value=str(link_metrics.get('gaps_over_150ms', 0))),
                KeyValue(key='decode_errors',
                         value=str(link_metrics.get('decode_errors', 0))),
                KeyValue(key='unknown_frames',
                         value=str(link_metrics.get('unknown_frames', 0))),
                KeyValue(key='tx_errors',
                         value=str(link_metrics.get('tx_errors', 0))),
                KeyValue(key='tick_overruns',
                         value=str(link_metrics.get('tick_overruns', 0))),
                KeyValue(key='bus_reopens',
                         value=str(can_info.get('reopens', 0))),
                KeyValue(key='channel', value=str(can_info.get('channel'))),
                KeyValue(key='drive_src', value=str(drive.get('src'))),
            ]
        array.status.append(link)
        self.diag_pub.publish(array)

    # ---- завершение ---------------------------------------------------------
    def close(self) -> None:
        # Моторы намеренно не трогаем: deadman демона коастит их через 0.5 с
        # тишины, а рестарт ROS-стека не должен дёргать ходовую.
        self._stop.set()
        try:
            self._sock.close()
        except OSError:
            pass
        self._rx_thread.join(timeout=2.0)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node: Optional[VescBridgeNode] = None
    try:
        node = VescBridgeNode()
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
