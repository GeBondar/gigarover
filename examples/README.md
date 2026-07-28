# Примеры управления GIGAROVER

Рабочие скрипты — каждый способ управления из [docs/driving.md](../docs/driving.md)
в виде кода. Все примеры с HTTP/UDP проверены на симуляторе ходовой
(`tools/motord/motord.py --sim`) — их можно гонять и без ровера.

| Скрипт | Канал | Откуда запускать | Зависимости |
| --- | --- | --- | --- |
| [http_drive.py](http_drive.py) | HTTP API :8765/:8767 | любое устройство в сети ровера | только Python 3 |
| [http_status.py](http_status.py) | HTTP API :8765/:8767 | любое устройство в сети ровера | только Python 3 |
| [udp_drive.py](udp_drive.py) | UDP API rover-motord :8460 | только на ровере (localhost) | только Python 3 |
| [ros2_cmd_vel.py](ros2_cmd_vel.py) | топик `/cmd_vel_teleop` | только на ровере (DDS в localhost) | ROS 2 Jazzy |

## Быстрый старт (телефон/ноутбук в Wi-Fi сети GIGAROVER)

```bash
# телеметрия: батарея, колёса, состояние CAN-линка
python3 http_status.py

# проехать вперёд 2 секунды на 0.4 м/с (колёса на подставке для первого раза!)
python3 http_drive.py forward --speed 0.4 --duration 2

# развернуться на месте
python3 http_drive.py turn --omega 1.0 --duration 1.6

# аварийный стоп
python3 http_drive.py stop
```

## Проверка без ровера (симулятор)

На любом компьютере с Python 3:

```bash
python3 ../tools/motord/motord.py --sim     # оставить работать
python3 http_drive.py --host 127.0.0.1 --port 8767 forward
python3 http_status.py --host 127.0.0.1 --port 8767
```

## Главное правило

**Одна команда не едет.** На каждом уровне стоит дедмен (0.25–0.5 с):
замолчал источник — моторы отпускаются. Поэтому все примеры шлют команды
потоком 20 Гц и заканчивают явным стопом. Пишете своё — делайте так же.
