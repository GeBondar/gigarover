#!/usr/bin/env python3
"""Управление GIGAROVER из ROS 2: публикация Twist в /cmd_vel_teleop.

Запускается НА РОВЕРЕ (DDS заперт в localhost — с внешнего ПК топики не
видны, см. docs/driving.md). Перед запуском:

    source /opt/ros/jazzy/setup.bash
    source ~/rover_ws/install/setup.bash
    python3 ~/rover_ws/examples/ros2_cmd_vel.py --vx 0.4 --duration 2.0
    python3 ~/rover_ws/examples/ros2_cmd_vel.py --wz 1.0 --duration 1.6

Почему поток, а не одна публикация: дедмены на каждом уровне (twist_mux
0.5 с, мост 0.5 с, демон 0.5 с) гасят одиночную команду через полсекунды.
Публикуем 20 раз в секунду, в конце — явный нулевой Twist.

Топики (приоритеты twist_mux): /cmd_vel_teleop (100, ручное управление,
перебивает всё) > /cmd_vel_test (75, тестовые скрипты) > /cmd_vel_nav
(50, Nav2). Для своих автономных программ используйте /cmd_vel_test —
тогда джойстик веб-морды всегда сможет вас перебить.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CmdVelStreamer(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("example_cmd_vel_streamer")
        self.publisher = self.create_publisher(Twist, topic, 10)

    def stream(self, vx: float, wz: float, duration: float,
               rate_hz: float) -> None:
        message = Twist()
        message.linear.x = vx
        message.angular.z = wz
        period = 1.0 / rate_hz
        deadline = time.monotonic() + duration
        self.get_logger().info(
            f"Еду: vx={vx} м/с, wz={wz} рад/с, {duration} с")
        while rclpy.ok() and time.monotonic() < deadline:
            self.publisher.publish(message)
            time.sleep(period)

    def stop(self) -> None:
        # Несколько нулевых Twist подряд — надёжнее одного (best effort QoS)
        for _ in range(5):
            self.publisher.publish(Twist())
            time.sleep(0.05)
        self.get_logger().info("Стоп.")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--vx", type=float, default=0.0, help="м/с, вперёд >0")
    parser.add_argument("--wz", type=float, default=0.0, help="рад/с, влево >0")
    parser.add_argument("--duration", type=float, default=2.0, help="секунды")
    parser.add_argument("--rate", type=float, default=20.0, help="Гц")
    parser.add_argument("--topic", default="/cmd_vel_teleop",
                        help="/cmd_vel_teleop | /cmd_vel_test | /cmd_vel")
    args = parser.parse_args()

    rclpy.init()
    node = CmdVelStreamer(args.topic)
    try:
        node.stream(args.vx, args.wz, args.duration, args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
