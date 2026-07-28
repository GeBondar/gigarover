# GIGAROVER — ROS 2 workspace

![GIGAROVER](docs/images/gigarover.jpg)

Рабочее пространство ROS 2 Jazzy большого четырёхколёсного ровера GIGAROVER
(скид-стир). Код основан на sverk_rover; ключевое отличие — ходовая часть:
четыре мотор-колеса PX60 с контроллерами VESC на общей CAN-шине 500 кбит/с,
подключённой к бортовому компьютеру Orange Pi 5 Pro через USB-CAN адаптер на
CH340 (протокол Seeedstudio, VID:PID `1a86:7523`).

## Документация для участников

| Документ | О чём |
| --- | --- |
| **[docs/driving.md](docs/driving.md)** | все способы ездить: веб-джойстик, HTTP API, ROS 2, UDP, маршруты, Nav2 |
| [examples/](examples/) | рабочие скрипты управления (проверены на симуляторе) |
| [docs/web_interfaces.md](docs/web_interfaces.md) | веб-морды со скриншотами: :8765, :8767, rosboard, настройка моторов |
| [docs/packages.md](docs/packages.md) | что за что отвечает: пакеты, узлы, топики, конфиги, порты |
| этот README (ниже) | прошивка платы, установка, настройка VESC, калибровка |

Быстрый старт на готовом ровере: подключиться к Wi-Fi `GIGAROVER`
(пароль `gigarover`) и открыть `http://10.42.0.1:8765` — страница
«Движение». Аварийный телеоп, работающий даже без ROS, —
`http://10.42.0.1:8767`.

Отличия от sverk_rover:

- ходовая — демон `rover-motord` (`tools/motord`, без ROS: единолично владеет
  CAN-адаптером, контур 50 Гц, SET_RPM на четыре VESC, телеметрия Status
  1/2/4/5, аварийный веб-телеоп на :8767) плюс тонкий ROS-мост
  `rover_vesc_driver/vesc_bridge_node` (публикация `/wheel/encoders`,
  `/battery/state`, `/diagnostics`) вместо serial-драйвера
  `rover_base_driver`;
- первичная настройка моторов — веб-приложение `rover_setup_web`
  (сканирование CAN, привязка колёс, тест хода, экспорт `motors.yaml`);
- конфиг робота `gigarover_v1.yaml` и профили запуска `gigarover*`;
- исключены пакеты малого ровера: LED-лента, октолайнер, аудио, дисплей,
  агент/MCP, fleet-мост.

## Образ ОС

Плата: Orange Pi 5 Pro (Rockchip RK3588S). Используется готовый server-образ
Ubuntu 24.04 из проекта ubuntu-rockchip (Joshua-Riek), v2.4.0:

```text
https://github.com/Joshua-Riek/ubuntu-rockchip/releases/download/v2.4.0/ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img.xz
https://github.com/Joshua-Riek/ubuntu-rockchip/releases/download/v2.4.0/ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img.xz.sha256
```

Почему именно он: ROS 2 Jazzy официально поддерживается только на
Ubuntu 24.04 (Tier 1), server-вариант без рабочего стола экономит ресурсы
платы, а сборки ubuntu-rockchip поддерживают Orange Pi 5 Pro «из коробки».

Проверка контрольной суммы и прошивка microSD:

```bash
wget https://github.com/Joshua-Riek/ubuntu-rockchip/releases/download/v2.4.0/ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img.xz
wget https://github.com/Joshua-Riek/ubuntu-rockchip/releases/download/v2.4.0/ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img.xz.sha256
sha256sum -c ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img.xz.sha256
```

Прошить можно Raspberry Pi Imager («Use custom image») или balenaEtcher —
оба принимают `.img.xz` напрямую, либо `dd`:

```bash
xz -dk ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img.xz
sudo dd if=ubuntu-24.04-preinstalled-server-arm64-orangepi-5-pro.img of=/dev/sdX bs=4M status=progress conv=fsync
```

### Первый вход

Логин/пароль по умолчанию: `ubuntu` / `ubuntu` (система сразу потребует
сменить пароль; если нет — смените вручную командой `passwd`). Затем задайте
имя хоста:

```bash
sudo hostnamectl set-hostname gigarover-01
```

### Wi-Fi

Вариант 1 — NetworkManager (ставится скриптом `install.sh`; на свежем образе
его может ещё не быть):

```bash
sudo nmcli device wifi connect "MySSID" password "MyPassword"
```

Вариант 2 — netplan (штатный для server-образа). Добавьте в
`/etc/netplan/50-cloud-init.yaml`:

```yaml
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      access-points:
        "MySSID":
          password: "MyPassword"
```

и примените: `sudo netplan apply`.

Вариант 3 — если блок `access-points` в netplan уже есть, SSID/пароль меняются
одной командой утилитой из workspace:

```bash
sudo python3 tools/rover-wifi-config.py apply wlan0 /etc/netplan/50-cloud-init.yaml "MySSID" "MyPassword"
```

### SSH

SSH-сервер (`openssh-server`) в server-образе уже установлен и стартует при
загрузке — отдельно включать ничего не нужно. Для первого подключения проще
всего воткнуть Ethernet (DHCP работает из коробки) или настроить Wi-Fi с
монитором/клавиатурой, затем узнать IP платы:

```bash
ip -4 addr show
```

(либо посмотрите выданный адрес в веб-интерфейсе роутера). Подключение с ПК:

```bash
ssh ubuntu@<IP-платы>
```

При первом входе система потребует сменить пароль `ubuntu` и разорвёт
сессию — подключитесь второй раз уже с новым паролем.

Чтобы не вводить пароль каждый раз, настройте вход по ключу. На Windows
(PowerShell) сгенерируйте ключ и скопируйте его на ровер (утилиты
`ssh-copy-id` в Windows нет, поэтому копируем через конвейер):

```powershell
ssh-keygen -t ed25519
type $env:USERPROFILE\.ssh\id_ed25519.pub | ssh ubuntu@<IP-платы> "mkdir -p ~/.ssh && cat >> ~/.ssh/authorized_keys && chmod 700 ~/.ssh && chmod 600 ~/.ssh/authorized_keys"
```

На Linux/macOS то же самое делает `ssh-copy-id ubuntu@<IP-платы>`.
Проверьте, что `ssh ubuntu@<IP-платы>` пускает без пароля, и только после
этого при желании отключите вход по паролю:

```bash
echo "PasswordAuthentication no" | sudo tee /etc/ssh/sshd_config.d/10-gigarover.conf
sudo systemctl restart ssh
```

Чтобы обращаться к роверу по имени `gigarover-01.local` вместо IP, поставьте
mDNS-демон:

```bash
sudo apt-get install -y avahi-daemon
```

Удобно также прописать алиас на рабочем ПК в `~/.ssh/config`
(на Windows — `C:\Users\<вы>\.ssh\config`):

```text
Host rover
    HostName gigarover-01.local
    User ubuntu
```

после чего работают короткие команды `ssh rover`, `scp -r rover_ws rover:~/`
и туннель для setup-web: `ssh -L 5000:127.0.0.1:5000 rover`.

## Установка

Скопируйте workspace на плату и запустите бутстрап:

```bash
# с рабочего ПК
scp -r rover_ws ubuntu@gigarover-01:~/
# либо git clone <репозиторий> ~/rover_ws на самой плате

ssh ubuntu@gigarover-01
cd ~/rover_ws
bash install.sh
```

Запускайте именно через `bash install.sh`: при копировании через `scp`
(особенно с Windows) бит исполняемости теряется, и `./install.sh` вернёт
`Permission denied`.

`install.sh` идемпотентен (можно перезапускать) и выполняет:

1. генерацию локали `en_US.UTF-8`;
2. подключение репозитория universe, установку curl/git;
3. подключение apt-источника ROS 2 (официальный deb `ros2-apt-source`);
4. установку apt-пакетов (таблица ниже);
5. `rosdep init/update` и `rosdep install --from-paths src --ignore-src -y
   --skip-keys "rover_waveshare_audio ament_pytest"`;
6. добавление пользователя в группу `dialout`;
7. установку правил udev (`/dev/rover_can`, защита от ModemManager);
8. удаление `brltty` (перехватывает CH340);
9. сборку: `colcon build --symlink-install`;
10. установку systemd-служб: `rover-motord` (демон ходовой, владелец
    USB-CAN, автозапуск включён), `rover-bringup` (автозапуск включён) и
    `rover-setup-web` (выключена, запускается вручную).

## Список зависимостей

Всё, что ставит `install.sh` через apt:

| Пакет | Зачем |
| --- | --- |
| `locales` | генерация локали en_US.UTF-8 (требование ROS 2) |
| `software-properties-common` | `add-apt-repository` для подключения universe |
| `curl`, `git`, `ca-certificates` | загрузка ros2-apt-source, работа с репозиториями |
| `ros2-apt-source` (deb с GitHub) | официальный apt-репозиторий ROS 2 |
| `ros-jazzy-ros-base` | ядро ROS 2 Jazzy (rclpy, ros2cli, DDS) без GUI |
| `ros-dev-tools` | инструменты разработки ROS (vcstool, rosdep и пр.) |
| `python3-colcon-common-extensions` | сборщик workspace `colcon` |
| `python3-rosdep` | установка зависимостей из package.xml |
| `ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup` | навигационный стек Nav2 |
| `ros-jazzy-slam-toolbox` | SLAM, построение и обновление карт |
| `ros-jazzy-robot-localization` | EKF-фьюжн колёсной одометрии и IMU |
| `ros-jazzy-twist-mux` | приоритизация источников `/cmd_vel` |
| `ros-jazzy-xacro`, `ros-jazzy-robot-state-publisher` | URDF-описание робота и TF |
| `ros-jazzy-rmw-fastrtps-cpp` | DDS-транспорт (Fast RTPS) |
| `python3-can` | CAN-шина VESC через seeedstudio-адаптер (драйвер и setup-web) |
| `python3-serial` | автодетект CH340, работа с последовательными портами |
| `python3-flask` | веб-приложение rover_setup_web |
| `python3-tornado`, `simplejpeg` (pip) | rosboard (см. `src/rosboard/requirements.txt`); `python3-simplejpeg` в apt noble нет — ставится `pip3 install --break-system-packages simplejpeg` |
| `python3-opencv` | rover_camera / rover_vision |
| `python3-yaml` | чтение конфигов |
| `python3-numpy` | одометрия, обработка изображений |
| `v4l-utils` | диагностика USB-камер (`v4l2-ctl`) |
| `network-manager` | Wi-Fi через `nmcli` |

Остальные зависимости, объявленные в `package.xml` пакетов workspace,
доустанавливает `rosdep install` (шаг 5 установки).

## Настройка VESC

### Требования к прошивке и настройкам контроллеров

Проверяется в VESC Tool на каждом из четырёх контроллеров:

- уникальный `VESC ID` (он же CAN ID). Стендовая конфигурация ровера:
  FL=57, FR=25, RL=92, RR=71;
- `CAN Baud Rate` = 500K — одинаковый на всех контроллерах;
- включён broadcast CAN Status Messages **1, 2, 4 и 5** с частотой
  **не ниже 20 Гц** (рекомендуется 50 Гц): App Settings → General →
  `Can Status Message Mode` / `Can Status Rate`. Без них драйвер не получит
  телеметрию: Status 1 — ERPM/ток/duty, Status 2 — ампер-часы,
  Status 4 — температуры и входной ток, Status 5 — тахометр и напряжение.
  Драйвер публикует `/wheel/encoders` в темпе прихода Status 5, поэтому
  выставленная частота статусов задаёт и частоту колёсной одометрии;
- прогнан мастер настройки мотора (FOC detection) под мотор-колёса PX60,
  выставлены лимиты токов. Пар полюсов у PX60 — 7: от этого зависит пересчёт
  ERPM в скорость и одометрия;
- батарея — 6S li-ion; лимиты напряжения в VESC настроены соответственно.

### Первичная настройка через rover-setup-web

Setup-web и демон ходовой не могут работать одновременно — оба владеют одним
USB-CAN адаптером. systemd-юниты объявлены конфликтующими, поэтому достаточно
одной команды (rover-motord и rover-bringup остановятся сами):

```bash
sudo systemctl start rover-setup-web
```

Приложение слушает только `127.0.0.1:5000`, поэтому с рабочего ПК нужен
SSH-туннель:

```bash
ssh -L 5000:127.0.0.1:5000 ubuntu@gigarover-01
```

Откройте `http://127.0.0.1:5000` и пройдите шаги:

1. **Scan** — поиск VESC ID на шине;
2. **Identify** — выбранный мотор крутится на 5 % duty, определите, какое это
   колесо (шасси — на подставке, колёса в воздухе);
3. **Assign** — назначьте позицию FL/FR/RL/RR и направление «вперёд»
   для каждого ID;
4. **Drive-тест** — проверка скид-стира с телеметрией;
5. проверьте кинематические параметры (радиус колеса, колея, редукция,
   пары полюсов) — в веб-приложении они нужны для корректного drive-теста;
6. **Export** — файлы `motors.yaml` и `kinematics.yaml` записываются в
   `~/rover_config` (каталог задаётся переменной `ROVER_CONFIG_DIR`,
   для служб она прописана в `/etc/default/rover-bringup`).

Важно: из экспорта bringup использует только `motors.yaml` (CAN ID колёс,
инверсии, bitrate). `kinematics.yaml` — справочный файл, драйвер и одометрия
его **не читают**: радиус колеса, редукцию и число пар полюсов меняйте только
в `src/rover_bringup/config/gigarover_v1.yaml` (секции `geometry`/`encoders`),
иначе после замены колеса или ремня драйвер и одометрия продолжат считать
по старым значениям.

После экспорта:

```bash
sudo systemctl stop rover-setup-web
sudo systemctl start rover-motord rover-bringup
```

Запуск вручную (без systemd):

```bash
source install/setup.bash
ROVER_CONFIG_DIR=~/rover_config ros2 run rover_setup_web setup_web
```

Переменная `ROVER_CONFIG_DIR` обязательна: без неё приложение экспортирует
конфиги в каталог пакета, откуда bringup их **не читает**, и экспорт молча
пропадёт. Проверяйте строку `output dir:` при старте приложения.

При старте bringup подхватывает `~/rover_config/motors.yaml` (CAN ID колёс,
инверсии, bitrate) поверх значений из `gigarover_v1.yaml`. Конкретный порт
(`channel: /dev/ttyUSB0`) из motors.yaml не фиксируется: номера `ttyUSB`
меняются между перезагрузками, поэтому bringup подставляет стабильный симлинк
`/dev/rover_can` (правило udev), а без него включает автопоиск CH340.

## Запуск

Ходовой владеет демон `rover-motord` (без ROS, автозапуск): он держит CAN,
контур 50 Гц, deadman и аварийный веб-телеоп на :8767. ROS-стек
(`rover-bringup`) ходит к нему мостом и может перезапускаться сколько
угодно — моторы при тишине команд коастятся демоном через 0.5 с. Телефон
через :8767 перехватывает управление у ROS в любой момент (приоритет
источника web). Откат на прямой драйвер: `base_driver.type: vesc_direct`
в `gigarover_v1.yaml` + `sudo systemctl disable --now rover-motord`.

Автозапуск через systemd ставится `install.sh` (профиль `gigarover`):

```bash
sudo systemctl start rover-motord rover-bringup
sudo systemctl stop rover-bringup            # демон продолжает работать
systemctl status rover-motord rover-bringup
journalctl -u rover-motord -f                # журнал ходовой + метрики линка
journalctl -u rover-bringup -f
```

Настройки службы — в `/etc/default/rover-bringup` (профиль, workspace,
`ROVER_CONFIG_DIR`, дополнительные аргументы запуска).

Ручной запуск:

```bash
cd ~/rover_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch rover_bringup robot.launch.py profile:=gigarover
```

Профили:

| Профиль | Состав |
| --- | --- |
| `gigarover` | база: VESC-драйвер, одометрия, описание, EKF, веб-морда, rosboard, twist_mux |
| `gigarover_mapping` | + лидар и SLAM (построение карты) |
| `gigarover_navigation` | + лидар и Nav2 (навигация по карте) |

Профили с лидаром при первом запуске: лидар ищется по сохранённой конфигурации
устройств `~/.config/rover/devices.json`. Пока её нет, bringup автоматически
переходит в режим полного сканирования портов (в логе — `device mode=full`);
явно этот режим включается аргументом:

```bash
ros2 launch rover_bringup robot.launch.py profile:=gigarover_mapping discovery_mode:=full
```

Интерактивная утилита `ros2 run rover_device_manager setup_devices` рассчитана
на малый ровер (требует serial-контроллер моторов Quad-MD и IMU) и на
GIGAROVER не применяется; для диагностики поиска лидара есть
`ros2 run rover_device_manager discover_devices --mode full --require-lidar
--no-require-motor`.

Порты:

| Сервис | Порт | Доступ |
| --- | --- | --- |
| веб-морда (rover_web) | 8765 | `http://gigarover-01:8765` |
| аварийный телеоп rover-motord | 8767 | `http://gigarover-01:8767` (работает и без ROS) |
| rosboard | 8888 | `http://gigarover-01:8888` |
| rover-setup-web | 5000 | только `127.0.0.1` — через SSH-туннель |
| UDP API rover-motord | 8460 | только `127.0.0.1` (мост ROS) |

## Калибровка

Геометрия и калибровочные параметры — в
`src/rover_bringup/config/gigarover_v1.yaml`; значения по умолчанию —
первичная прикидка, всё, что требует обмера, помечено комментариями:

- `wheel_radius_m` 0.0825, `track_width_m` 0.40, `wheelbase_m` 0.50 —
  обмерить на собранном шасси;
- `encoders`: `encoder_lines` = 6 × пар полюсов = 42,
  `reduction_ratio` = передаточное отношение 3.0 (ремень HTD-5M) —
  менять только при замене мотора или редукции. Менять именно здесь:
  `kinematics.yaml` из rover-setup-web — справочный, драйвер и одометрия
  его не читают;
- `wheel_odometry`: `yaw_multiplier` калибруется на скид-стире по
  фактическому развороту (колёса проскальзывают).

Проверка одометрии:

```bash
ros2 topic echo /wheel/encoders
ros2 topic echo /odom
```

Проехать по прямой 2 м — сверить пройденную дистанцию (корректируется
`wheel_radius_m`); развернуться на 360° на месте — подобрать
`yaw_multiplier`. После правок конфига:

```bash
sudo systemctl restart rover-bringup
```

## Устранение неполадок

**Адаптер CH340 не виден (нет `/dev/ttyUSB0`, `/dev/rover_can`)**

- `lsusb | grep 1a86` — виден ли адаптер по USB (кабель, порт);
- `brltty` перехватывает CH340: `sudo apt-get purge -y brltty`
  (install.sh делает это автоматически);
- пользователь не в группе `dialout`: `id -nG | grep dialout`; после
  `usermod -aG dialout` нужен перелогин или перезагрузка;
- ModemManager опрашивает порт: правило
  `/etc/udev/rules.d/99-gigarover.rules` выставляет
  `ID_MM_DEVICE_IGNORE=1` — проверьте, что оно установлено, и
  переподключите адаптер.

**Колёса крутятся не в ту сторону**

- поправьте `invert` соответствующего колеса в
  `~/rover_config/motors.yaml` (или `wheel_inverts` в
  `gigarover_v1.yaml`) и перезапустите bringup;
- либо повторите identify/assign в rover-setup-web.

**Нет телеметрии (`/wheel/encoders` с `valid: false`, в `/diagnostics` STALE)**

- сначала проверьте демона: `systemctl status rover-motord`,
  `journalctl -u rover-motord -f`, `curl -s localhost:8767/api/health` —
  в `/diagnostics` статус `vesc_can_link` показывает state линка и метрики
  (rx Гц, gap'ы, ошибки декодирования, переоткрытия шины);
- в VESC Tool включите CAN Status Messages 1, 2, 4, 5 с частотой ≥ 20 Гц
  на всех контроллерах (рекомендуется 50 Гц; без Status 5 не будет
  одометрии вообще);
- проверьте одинаковый `CAN Baud Rate` 500K и уникальные VESC ID;
- убедитесь, что rover-setup-web и rover-can-teleop остановлены — они
  забирают адаптер у rover-motord (юниты конфликтующие, но проверить
  не вредно: `systemctl is-active rover-setup-web rover-can-teleop`).

**Ошибки / шум на CAN-шине**

- терминация: резисторы 120 Ом на обоих концах шины (мультиметр между
  CANH и CANL при выключенном питании должен показывать ~60 Ом);
- витая пара для CANH/CANL, минимальная длина отводов к контроллерам;
- общий GND между USB-CAN адаптером и VESC.
