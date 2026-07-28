# Запасной телеоп GIGAROVER: CAN без ROS

> **УСТАРЕВАЕТ.** Штатно ходовой теперь владеет постоянный демон
> [`rover-motord`](../motord/README.md) (`tools/motord`) — тот же код,
> выросший из этого телеопа, плюс UDP API для ROS-моста и аварийный
> веб-телеоп на **:8767**, работающий параллельно с ROS. Этот каталог
> сохранён как замороженный последний резерв: юнит `rover-can-teleop`
> останавливает motord/bringup/setup-web через `Conflicts=` и поднимает
> старую автономную схему на :8765.

Автономный фолбэк на случай проблем с ROS 2: один Python-процесс поднимает
управление VESC-ходовой напрямую по CAN, отдаёт веб-морду для телефона и
HTTP API. Никаких зависимостей от ROS — только `python-can`, `pyserial`,
`PyYAML` (на ровере уже стоят).

CAN-цикл — точный порт боевого `rover_vesc_driver` (та же логика, что
проверена тестами из `../drivetrain_tests`): TX SET_RPM по расписанию 50 Гц,
дренаж RX пачками ≤32 кадров, скид-стир микширование с пропорциональным
масштабированием, слю-лимиты разгона (торможение 2×), deadman 0.5 с (коаст
при молчании команд), подпор `min_erpm=900` (зона срыва sensorless), жёсткий
предел 4000 eRPM, переоткрытие шины с бэкоффом, ток 0 всем при любом выходе.

## Состав

| Файл | Что это |
|---|---|
| `can_teleop.py` | Всё приложение: CAN-драйвер + HTTP-сервер |
| `web/index.html` | Веб-морда (телефон/ноутбук), самодостаточная |
| `rover-can-teleop.service` | systemd-юнит с `Conflicts=` на ROS-службы |
| `install.sh` | Установка юнита на ровере |

## Заливка на ровер и установка

```bash
# с рабочей машины (ssh-алиас gigarover уже настроен):
rsync -av rover_ws/tools/can_teleop/ gigarover:rover_ws/tools/can_teleop/
ssh gigarover 'bash rover_ws/tools/can_teleop/install.sh'
```

## Запуск

**Владелец CAN-адаптера всегда один.** Скрипт сам отказывается стартовать,
пока активны `rover-bringup`/`rover-setup-web` (обход: `--force`).

Через systemd (рекомендуется — `Conflicts=` сам останавливает ROS-службы):

```bash
sudo systemctl start rover-can-teleop    # переключиться на фолбэк
sudo systemctl start rover-bringup       # вернуться на ROS-стек
```

Руками (для отладки):

```bash
sudo systemctl stop rover-bringup rover-setup-web
python3 ~/rover_ws/tools/can_teleop/can_teleop.py
```

Телефон в Wi-Fi сети `GIGAROVER`: **http://10.42.0.1:8765** (или
`http://rover.go:8765`). Из локалки по Ethernet: `http://192.168.2.72:8765`.

Флаги: `--host`, `--port`, `--channel /dev/rover_can`, `--motors-config`,
`--force`. Прочие параметры — константы `DEFAULTS` в начале `can_teleop.py`
(значения совпадают с `gigarover_v1.yaml`).

## Конфигурация колёс

Раскладка моторов и канал берутся из `~/rover_config/motors.yaml` — тот же
файл и те же правила, что у ROS-стека (`ROVER_CONFIG_DIR` учитывается;
волатильные `/dev/ttyUSB*` из файла заменяются на стабильный симлинк
`/dev/rover_can` из udev-правила). Порядок колёс везде: FL, FR, RL, RR.

## Веб-морда

- Джойстик (тач): вверх — вперёд, вправо — поворот направо. Отпустил — стоп.
- Слайдеры ограничения скорости (м/с) и поворота (рад/с), запоминаются.
- WASD/стрелки с ноутбука, пробел — STOP.
- STOP: мгновенный коаст + блокировка движения 0.75 с.
- Телеметрия: батарея (В, %, Вт), по колёсам м/с, eRPM, температура, ток,
  частота Status1, свежесть линка.
- Обрыв Wi-Fi/сворачивание страницы безопасны: сервер глушит моторы по
  deadman 0.5 с, страница дополнительно шлёт стоп при уходе в фон.

## HTTP API

Формы запросов совместимы с `/api/drive*` веб-морды ROS-стека. CORS открыт
(`Access-Control-Allow-Origin: *`) — можно дёргать с любого хоста в сети.

| Метод | Путь | Тело | Что делает |
|---|---|---|---|
| GET | `/api/health` | — | Краткий статус: аптайм, линк CAN |
| GET | `/api/status` | — | Полный снимок: CAN, батарея, колёса, команда |
| GET | `/api/drive` | — | Лимиты/дефолты/последняя команда (форма как в ROS-шлюзе) |
| POST | `/api/drive/command` | `{"linear_x": м/с, "angular_z": рад/с}` | Команда движения; слать ≥2 Гц (deadman 0.5 с), обычно 10 Гц |
| POST | `/api/drive/stop` | `{}` | Мягкий стоп (цель 0, съезд по слю-лимиту) |
| POST | `/api/stop` | `{}` | Аварийный стоп: мгновенный коаст + блокировка 0.75 с |

Примеры:

```bash
# ехать вперёд 0.5 м/с (повторять каждые ~100 мс, иначе deadman остановит)
curl -X POST http://10.42.0.1:8765/api/drive/command \
  -H 'Content-Type: application/json' -d '{"linear_x": 0.5, "angular_z": 0}'

# крутиться на месте влево 1 рад/с
curl -X POST http://10.42.0.1:8765/api/drive/command \
  -H 'Content-Type: application/json' -d '{"linear_x": 0, "angular_z": 1.0}'

# аварийный стоп
curl -X POST http://10.42.0.1:8765/api/stop -H 'Content-Type: application/json' -d '{}'

# телеметрия
curl http://10.42.0.1:8765/api/status | python3 -m json.tool
```

Конвенция знаков — ROS: `linear_x > 0` вперёд, `angular_z > 0` влево (CCW).
Лимиты сервера: `linear_x` ±1.5 м/с, `angular_z` ±3.0 рад/с, колесо ≤1.6 м/с
(превышение безопасно — команда масштабируется пропорционально).

В `/api/status` по каждому колесу: `erpm`, `measured_mps`, `duty`,
`motor_current`, `input_current`, `temp_fet`, `temp_motor`, `v_in`, `tacho`
(знаки скорректированы по `invert`), `status1_hz`/`status5_hz` (окно 5 с),
`max_gap_ms`/`gaps_over_150ms` — диагностика провалов линка как в
`can_health.py`.

## Безопасность

- Deadman: нет команд 0.5 с → ток 0 всем колёсам (коаст). Всегда, без опций.
- Любой выход процесса (Ctrl+C, SIGTERM, падение) → трижды ток 0 всем.
- `systemctl stop rover-can-teleop` шлёт SIGINT → штатный коаст (юнит).
- Слю-лимит разгона 1.5 м/с², жёсткие клампы eRPM.
- Колёса при первом запуске вывесить — как при любом тесте ходовой.
