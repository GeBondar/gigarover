# Пакеты и структура workspace

Справочник по всем составным частям GIGAROVER: что за что отвечает, какие
узлы и топики, где конфиги. Способы управления — в [driving.md](driving.md),
веб-интерфейсы — в [web_interfaces.md](web_interfaces.md).

## Карта репозитория

```
rover_ws/
├── src/                ROS 2-пакеты (см. таблицу ниже)
├── tools/
│   ├── motord/         демон ходовой (без ROS) — владелец CAN-шины
│   ├── can_teleop/     замороженный резервный телеоп (без ROS)
│   └── drivetrain_tests/  диагностика CAN/VESC
├── deploy/
│   ├── systemd/        юниты rover-motord / rover-bringup / rover-setup-web
│   ├── udev/           /dev/rover_can, защита от ModemManager
│   └── sudoers/        правило для rover-wifi-config
├── docs/               эта документация
├── examples/           рабочие примеры управления
└── install.sh          бутстрап чистой платы «под ключ»
```

## Сводная таблица пакетов

| Пакет | Узлы / артефакты | Назначение |
| --- | --- | --- |
| `rover_bringup` | launch + конфиги | точка входа: профили запуска, вся конфигурация |
| `rover_vesc_driver` | `vesc_bridge_node`, `vesc_driver_node` | связь ROS ↔ ходовая |
| `rover_wheel_odometry` | `wheel_odometry_node` | колёсная одометрия из энкодеров |
| `rover_web` | `web_gateway_node` (:8765) | веб-морда, HTTP API, исполнитель маршрутов |
| `rover_interfaces` | 6 msg + 6 srv | собственные типы сообщений |
| `rover_description` | URDF/xacro, RViz-виды | геометрия робота, TF-дерево |
| `rover_navigation` | `rover_map` + launch Nav2/SLAM | автономная навигация, карты |
| `rover_lidar_filter` | `lidar_footprint_filter` | вырезает корпус ровера из скана |
| `rover_setup_web` | `setup_web` (Flask, :5000) | первичная настройка моторов |
| `rover_device_manager` | `discover_devices`, `setup_devices` | автопоиск serial-устройств (лидар) |
| `rover_camera` | `usb_camera_node` | USB-камера (опция) |
| `rover_vision` | `camera_detector_node` | детекция объектов (опция) |
| `rover_imu` | `yahboom_imu_node` | IMU (на GIGAROVER не установлен) |
| `rosboard` | `rosboard_node` (:8888) | вендорный веб-просмотрщик топиков |
| `sllidar_ros2` | `sllidar_node` | вендорный драйвер лидара Slamtec C1 |

## Как собирается конфигурация

Порядок разрешения (каждый следующий слой перекрывает предыдущий):

1. **Профиль** `src/rover_bringup/config/profiles/<имя>.yaml` — какие
   компоненты включены. Рабочие профили GIGAROVER:

   | Профиль | Состав |
   | --- | --- |
   | `gigarover` | ходовая, одометрия, EKF, описание, twist_mux, веб, rosboard |
   | `gigarover_mapping` | + лидар и SLAM Toolbox (построение карты) |
   | `gigarover_navigation` | + лидар и Nav2 (езда по карте) |

2. **Компоненты** `config/components/*.yaml` — параметры узлов
   (например `web.yaml` — порт и лимиты веб-морды, `twist_mux.yaml` —
   приоритеты источников команд).
3. **Конфиг робота** `config/gigarover_v1.yaml` — физика конкретной
   машины: геометрия (`wheel_radius_m` 0.0825, `track_width_m` 0.40,
   `wheelbase_m` 0.50), энкодеры (`encoder_lines` 42 = 6 × 7 пар полюсов,
   `reduction_ratio` 3.0), ходовая `base_driver` (тип `vesc`, CAN ID
   колёс, лимиты eRPM/скоростей/ускорений).
4. **`config/topics.yaml`** — единый реестр имён топиков и TF-фреймов;
   узлы получают имена оттуда, в коде хардкода нет.
5. **`~/rover_config/motors.yaml`** (экспорт rover-setup-web, каталог
   задаётся `$ROVER_CONFIG_DIR`) — фактическая раскладка: CAN ID каждого
   колеса, инверсии, битрейт. Подмешивается поверх `gigarover_v1.yaml`
   при каждом старте и bringup, и демона motord.

> Важно: `kinematics.yaml` из setup-web — справочный, его никто не
> читает. Радиус колеса, редукция, пары полюсов правятся только в
> `gigarover_v1.yaml`.

## Ключевые пакеты подробно

### rover_bringup — точка входа

Единственный launch, который нужен в жизни:

```bash
ros2 launch rover_bringup robot.launch.py profile:=gigarover
```

`robot.launch.py` читает профиль, поднимает включённые компоненты,
подмешивает `motors.yaml`, пробрасывает геометрию в URDF и одометрию.
Полезные аргументы: `profile:=`, `discovery_mode:=full` (первый запуск
с лидаром, пока нет `~/.config/rover/devices.json`), `use_nav2:=`,
`use_slam:=` (перекрывают профиль). В штатной работе всё это делает
systemd-служба `rover-bringup` (настройки — `/etc/default/rover-bringup`:
`ROVER_PROFILE`, `ROVER_CONFIG_DIR`, `MOTORD_OPTS`).

### rover_vesc_driver — ROS ↔ ходовая

Два узла, включается по `base_driver.type` в `gigarover_v1.yaml`:

- **`vesc_bridge_node`** (тип `vesc`, штатный) — тонкий мост к демону
  `rover-motord` по UDP `127.0.0.1:8460`. Подписка `/cmd_vel`
  (QoS keep_last(1), best effort) → датаграммы `drive`; поток `state`
  демона → `/wheel/encoders` (`rover_interfaces/WheelEncoders`),
  `/battery/state` (`sensor_msgs/BatteryState`), `/diagnostics`
  (статусы `vesc_fl…rr` + `vesc_can_link` с метриками линка). Демон
  недоступен → ERROR в диагностике, моторы не трогает (их отпустит
  дедмен демона).
- **`vesc_driver_node`** (тип `vesc_direct`, откат) — прежний прямой
  драйвер CAN в одном процессе с ROS. Требует **выключенного** motord
  (`sudo systemctl disable --now rover-motord`), иначе два писателя
  на шине.

Пересчёт скоростей: колесо ↔ eRPM через редукцию 3.0 и 7 пар полюсов;
энкодер — тахометр VESC, 126 отсчётов на оборот колеса. Команды ниже
`min_erpm` (900) подпираются — сверхмалых скоростей у sensorless-моторов
нет.

### rover_wheel_odometry + EKF

`wheel_odometry_node`: `/wheel/encoders` → `/wheel/odometry`
(`nav_msgs/Odometry`), скид-стир кинематика, `yaw_multiplier` —
калибровка проскальзывания на развороте. Дальше `robot_localization`
(`ekf_node`) фьюзит в `/odom` (на GIGAROVER — конфиг
`ekf_wheel_only.yaml`, IMU нет).

### rover_web — веб-морда и HTTP API

`web_gateway_node` — HTTP-сервер :8765 (ThreadingHTTPServer) + ROS-узел
в одном процессе. Публикует `Twist` в топик из параметра
`command_topic` (в профилях с twist_mux — `/cmd_vel_teleop`) таймером
20 Гц, дедмен 0.25 с. Подписки: `/odom`, `/wheel/odometry`, `/imu/data`,
`/diagnostics`. HTTP API (~40 endpoints): движение (`/api/drive/*`),
статус (`/api/status`, `/api/system`), маршруты (`/api/plans*`,
`/api/motion/*`), периферия (`/api/camera|lidar|led_strip|octoliner/*`),
произвольные топики/сервисы ROS (`/api/ros/*`). Статика UI —
`web/index.html` + `assets/app.js`. Планы маршрутов —
`~/.local/share/sverh-rover-web/plans/*.yaml` (при первом старте
сеются примеры из пакета).

### rover_interfaces — типы сообщений

Используются на GIGAROVER: `msg/WheelEncoders` (счётчики + м/с ×4,
`sequence`, `valid`), `msg/WheelCommand`, `srv/GetFrame` (кадр камеры).
Остальные (LED, октолайнер, голос) — наследие малого ровера, узлов под
них здесь нет.

### rover_navigation — Nav2 и SLAM

Launch: `navigation.launch.py` (map_server + AMCL + Nav2-стек, команды
идут в `/cmd_vel_nav`), `slam.launch.py` (slam_toolbox async),
`update_map.launch.py` (дозаписать существующую карту). Параметры под
GIGAROVER: `rover_bringup/config/navigation/nav2_params_gigarover.yaml`
(DWB: max 1.0 м/с, 2.0 рад/с; footprint 0.68×0.50 м; скан —
`/scan_filtered`) и `slam_toolbox_params_gigarover.yaml` (разрешение
0.05 м, дальность 8 м). Карты: `ros2 run rover_navigation rover_map
save|list|status|use` — архив в `maps/`, текущая — `maps/current/`.

### rover_setup_web — первичная настройка моторов

Flask-приложение (не ROS-узел), `127.0.0.1:5000`, доступ через
SSH-туннель. Сканирует CAN, крутит мотор для опознания колеса,
назначает позиции FL/FR/RL/RR и направления, драйв-тест, экспорт
`motors.yaml` в `$ROVER_CONFIG_DIR`. Владеет CAN-адаптером — systemd-юнит
объявлен конфликтующим с motord/bringup: `sudo systemctl start
rover-setup-web` сам останавливает боевой стек (и наоборот).

### Вендорные пакеты

- **rosboard** (dheera/rosboard, BSD) — просмотр любых топиков в браузере,
  :8888. Скриншоты — `src/rosboard/screenshots/`.
- **sllidar_ros2** (Slamtec, BSD) — драйвер лидара C1, публикует `/scan`;
  дальше `rover_lidar_filter` вырезает точки внутри корпуса →
  `/scan_filtered` для SLAM/Nav2.

## tools/ — вне ROS

### motord — демон ходовой (главное звено)

`tools/motord/motord.py`, служба `rover-motord`. Единственный владелец
USB-CAN. Контур 50 Гц: скид-стир микширование, слю-лимиты, `min_erpm`,
дедмен по источнику, арбитраж `web` (телефон :8767) > `ros` (мост).
RX — отдельный блокирующий поток; метрики линка (частоты статусов,
gap'ы, ошибки) — в journal каждые 30 с и в `/diagnostics` через мост.
systemd `Type=notify` + watchdog 15 с. Интерфейсы: HTTP `:8767`
(телеоп + API), UDP `127.0.0.1:8460` (мост/скрипты). Режим `--sim` —
встроенный симулятор четырёх VESC, работает на любом ПК (примеры из
`examples/` можно отлаживать без ровера). Подробный протокол —
[tools/motord/README.md](../tools/motord/README.md).

### can_teleop — резерв

Предшественник motord, заморожен. Один процесс: CAN-ходовая + веб-телеоп
:8765. Юнит `rover-can-teleop` ставится отдельным `tools/can_teleop/
install.sh`, не в автозапуске, конфликтует со всеми владельцами CAN.

### drivetrain_tests — диагностика ходовой

Прямые CAN-скрипты (боевой стек остановить; CAN ID в скриптах — 32/34/
104/114 под текущий комплект моторов): `can_health.py` — ступенчатый
профиль eRPM (1500→2500→1200→800→1500) с записью телеметрии в CSV и
вердиктами; `reversal_test.py` — ловля самопроизвольных реверсов;
`coast_test.py` — чистота приёма на выбеге; `tx_flood_test.py` — влияние
собственной передачи на приём; `ros_soak_test.py` — сквозной тест через
ROS (`/cmd_vel_teleop` → фактические eRPM из `/diagnostics`);
`analyze_*.py` — офлайн-разбор CSV.

## deploy/ — служебная обвязка

| Юнит | Автозапуск | Роль |
| --- | --- | --- |
| `rover-motord` | ✅ | демон ходовой, владелец CAN, телеоп :8767 |
| `rover-bringup` | ✅ | ROS-стек (`robot.launch.py`, профиль из env) |
| `rover-setup-web` | ручной | настройка моторов :5000; Conflicts= с motord и bringup |
| `rover-can-teleop` | ручной | резервный телеоп; Conflicts= со всеми |

`deploy/systemd/install.sh` ставит первые три (вызывается из общего
`install.sh`), подставляя пользователя и путь workspace. Правила udev
(`deploy/udev/99-gigarover.rules`): стабильный симлинк `/dev/rover_can`
для CH340, запрет ModemManager, отключение USB-автосна.

## Порты — сводка

| Порт | Сервис | Доступ снаружи |
| --- | --- | --- |
| 8765 | веб-морда `rover_web` (или can_teleop в резервном режиме) | ✅ |
| 8767 | аварийный телеоп + API `rover-motord` | ✅ |
| 8888 | rosboard | ✅ |
| 5000 | rover-setup-web | только localhost (SSH-туннель) |
| 8460/udp | UDP API rover-motord | только localhost |

HTTP-порты без аутентификации — защита ровера = WPA2-пароль точки
GIGAROVER. Не поднимайте точку с дефолтным паролем на публичных
мероприятиях без необходимости.
