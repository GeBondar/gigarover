#!/usr/bin/env python3
"""Сквозной соак-тест через ROS-стек: публикуем профиль в /cmd_vel_teleop
(тот же путь, что у веб-морды: twist_mux -> vesc_driver) и параллельно
слушаем /diagnostics, сверяя фактические eRPM колёс с ожиданием.

Профиль с нулевым суммарным смещением (безопасно на земле):
вращение влево/вправо и короткие вперёд/назад.
"""
import math
import sys
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Twist
from rclpy.node import Node

TRACK = 0.40
ERPM_PER_MPS = 3.0 * 60.0 * 7.0 / (2.0 * math.pi * 0.0825)   # ~2431
MIN_ERPM = 900.0
WHEELS = ("vesc_fl", "vesc_fr", "vesc_rl", "vesc_rr")

# (имя, vx, wz, длительность_с)
PROFILE = [
    ("rot_left",  0.0,  2.0, 3.0),
    ("rot_right", 0.0, -2.0, 3.0),
    ("fwd",       0.5,  0.0, 2.0),
    ("back",     -0.5,  0.0, 2.0),
    ("stop",      0.0,  0.0, 1.0),
]


def expected_abs_erpm(vx, wz):
    v_l = abs(vx - wz * TRACK / 2.0)
    v_r = abs(vx + wz * TRACK / 2.0)
    out = []
    for v in (v_l, v_r, v_l, v_r):     # FL FR RL RR
        e = v * ERPM_PER_MPS
        if 0.0 < e < MIN_ERPM:
            e = MIN_ERPM
        out.append(e)
    return out


class Soak(Node):
    def __init__(self):
        super().__init__("soak_test")
        self.pub = self.create_publisher(Twist, "/cmd_vel_teleop", 10)
        self.samples = []          # (t, {wheel: erpm})
        self.create_subscription(DiagnosticArray, "/diagnostics", self._diag, 10)

    def _diag(self, msg):
        rec = {}
        for st in msg.status:
            if st.name in WHEELS:
                for kv in st.values:
                    if kv.key == "erpm":
                        try:
                            rec[st.name] = float(kv.value)
                        except ValueError:
                            pass
        if rec:
            self.samples.append((time.time(), rec))


def main():
    rclpy.init()
    node = Soak()
    time.sleep(1.5)                 # прогрев discovery
    seg_windows = []
    for name, vx, wz, dur in PROFILE:
        t0 = time.time()
        msg = Twist()
        msg.linear.x = vx
        msg.angular.z = wz
        while time.time() - t0 < dur:
            node.pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.05)
        seg_windows.append((name, vx, wz, t0 + dur * 0.4, t0 + dur))
    stop = Twist()
    for _ in range(5):
        node.pub.publish(stop)
        rclpy.spin_once(node, timeout_sec=0.05)
    t_end = time.time() + 1.0
    while time.time() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    print(f"диагностических сэмплов: {len(node.samples)}")
    problems = 0
    for name, vx, wz, w0, w1 in seg_windows:
        exp = expected_abs_erpm(vx, wz)
        got = {w: [] for w in WHEELS}
        for t, rec in node.samples:
            if w0 <= t <= w1:
                for w, e in rec.items():
                    got[w].append(abs(e))
        cells = []
        for i, w in enumerate(WHEELS):
            if got[w]:
                mean = sum(got[w]) / len(got[w])
                mark = ""
                if exp[i] > 0 and (mean < exp[i] * 0.55 or mean > exp[i] * 1.6):
                    mark = " !"
                    problems += 1
                if exp[i] == 0 and mean > 250:
                    mark = " !"
                    problems += 1
                cells.append(f"{w[5:]}={mean:5.0f}{mark}")
            else:
                cells.append(f"{w[5:]}=  n/a")
        print(f"  {name:9s} ожид |eRPM| FL/FR={exp[0]:.0f}/{exp[1]:.0f} "
              f"факт: " + "  ".join(cells))
    node.destroy_node()
    rclpy.shutdown()
    print("ИТОГ:", "есть расхождения (см. !)" if problems else
          "✓ все колёса следуют командам через полный ROS-стек")
    return 1 if problems else 0


if __name__ == "__main__":
    sys.exit(main())
