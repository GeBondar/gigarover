#!/usr/bin/env bash
#
# GIGAROVER: бутстрап чистой Ubuntu 24.04 (arm64, Orange Pi 5 Pro) до собранного
# workspace ROS 2 Jazzy с автозапуском.
#
# Запускать от обычного пользователя с правами sudo, из корня workspace:
#
#   cd ~/rover_ws && bash install.sh
#
# Именно `bash install.sh`: после копирования с другой машины (scp/архив)
# бит исполняемости может быть потерян, и `./install.sh` даст Permission denied.
#
# Скрипт идемпотентен: уже выполненные шаги пропускаются, повторный запуск
# безопасен. Все вопросы apt подавлены (DEBIAN_FRONTEND=noninteractive).

set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "${SCRIPT_DIR}"

export DEBIAN_FRONTEND=noninteractive
TARGET_USER="${SUDO_USER:-$(id -un)}"

section() { echo; echo "==== $* ===="; }
apt_install() { sudo DEBIAN_FRONTEND=noninteractive apt-get install -y "$@"; }
apt_update() { sudo DEBIAN_FRONTEND=noninteractive apt-get update; }

# --- 1. Локаль en_US.UTF-8 (требование ROS 2) --------------------------------
section "Локаль en_US.UTF-8"
if locale -a 2>/dev/null | grep -qiE '^en_US\.utf-?8$'; then
  echo "локаль уже есть — пропуск"
else
  apt_update
  apt_install locales
  sudo locale-gen en_US en_US.UTF-8
  sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
fi
export LANG=en_US.UTF-8

# --- 2. Репозиторий universe + базовые утилиты -------------------------------
section "Репозиторий universe, curl, git"
apt_update
apt_install software-properties-common curl git ca-certificates
if apt-cache policy | grep -q "/universe"; then
  echo "universe уже подключён — пропуск"
else
  sudo add-apt-repository -y universe
  apt_update
fi

# --- 3. apt-источник ROS 2 (пакет ros2-apt-source) ---------------------------
section "apt-источник ROS 2"
if dpkg -s ros2-apt-source >/dev/null 2>&1; then
  echo "ros2-apt-source уже установлен — пропуск"
else
  UBUNTU_CODENAME="$(. /etc/os-release && echo "${VERSION_CODENAME}")"
  ROS_APT_SOURCE_VERSION="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
    | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
  deb_tmp="$(mktemp /tmp/ros2-apt-source.XXXXXX.deb)"
  curl -fL -o "${deb_tmp}" \
    "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${UBUNTU_CODENAME}_all.deb"
  apt_install "${deb_tmp}"
  rm -f "${deb_tmp}"
  apt_update
fi

# --- 4. Пакеты apt ------------------------------------------------------------
section "Пакеты apt (ROS 2 Jazzy, Nav2, SLAM, python-зависимости)"
apt_install \
  ros-jazzy-ros-base \
  ros-dev-tools \
  python3-colcon-common-extensions \
  python3-rosdep \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-twist-mux \
  ros-jazzy-xacro \
  ros-jazzy-robot-state-publisher \
  ros-jazzy-rmw-fastrtps-cpp \
  python3-can \
  python3-serial \
  python3-flask \
  python3-tornado \
  python3-pip \
  python3-opencv \
  python3-yaml \
  python3-numpy \
  v4l-utils \
  network-manager

# simplejpeg (нужен rosboard для сжатия картинок) в apt Ubuntu 24.04 отсутствует —
# ставим через pip в системный python (на роботе это осознанно).
if python3 -c "import simplejpeg" 2>/dev/null; then
  echo "simplejpeg уже установлен — пропуск"
else
  sudo pip3 install --break-system-packages simplejpeg
fi

# --- 5. rosdep ----------------------------------------------------------------
section "rosdep (зависимости пакетов workspace)"
sudo rosdep init 2>/dev/null || echo "rosdep уже инициализирован — пропуск"
# rosdep тянет индексы с github — при сетевом сбое НЕ роняем установку:
# все системные зависимости уже поставлены явным apt-списком выше,
# rosdep здесь только подстраховка на будущие пакеты.
rosdep update || echo "ВНИМАНИЕ: rosdep update не прошёл (сеть?) — продолжаем без него"
set +ue
source /opt/ros/jazzy/setup.bash
set -ue
# ament_pytest: несуществующий rosdep-ключ (test_depend в rover_lidar_filter,
# скопирован из upstream) — пропускаем, тесты не собираем.
rosdep install --from-paths src --ignore-src -y \
  --skip-keys "rover_waveshare_audio ament_pytest" \
  || echo "ВНИМАНИЕ: rosdep install не прошёл — полагаемся на явный apt-список выше"

# --- 6. Группа dialout --------------------------------------------------------
section "Группа dialout для пользователя ${TARGET_USER}"
if id -nG "${TARGET_USER}" | grep -qw dialout; then
  echo "пользователь уже в dialout — пропуск"
else
  sudo usermod -aG dialout "${TARGET_USER}"
  echo "добавлен в dialout — после установки перелогиньтесь или перезагрузитесь"
fi

# --- 7. Правила udev ----------------------------------------------------------
section "Правила udev (/dev/rover_can, защита от ModemManager)"
sudo install -m 0644 deploy/udev/99-gigarover.rules /etc/udev/rules.d/99-gigarover.rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# --- 8. brltty ----------------------------------------------------------------
section "Удаление brltty (перехватывает CH340)"
sudo DEBIAN_FRONTEND=noninteractive apt-get purge -y brltty 2>/dev/null \
  || echo "brltty не установлен либо не удалился — не критично"

# --- 9. Сборка workspace ------------------------------------------------------
section "Сборка workspace (colcon build --symlink-install)"
colcon build --symlink-install

# --- 10. Службы systemd -------------------------------------------------------
section "Службы systemd"
# Через bash: после scp с другой машины exec-бит на скрипте может отсутствовать.
bash "${SCRIPT_DIR}/deploy/systemd/install.sh"

# --- Готово -------------------------------------------------------------------
section "Готово"
cat <<EOF

============================================================
Установка завершена. Дальнейшие шаги:

  1. Перезагрузитесь (или перелогиньтесь), чтобы применилась
     группа dialout:
       sudo reboot

  2. Подключите USB-CAN адаптер (CH340) — он появится как
     /dev/ttyUSB0 и /dev/rover_can.

  3. Первичная настройка VESC (однократно):
       sudo systemctl start rover-setup-web
     (motord и bringup остановятся сами через Conflicts=).
     Веб-интерфейс слушает только 127.0.0.1:5000 — откройте
     туннель со своего ПК:
       ssh -L 5000:127.0.0.1:5000 ${TARGET_USER}@<ip-ровера>
     и зайдите на http://127.0.0.1:5000. После экспорта
     motors.yaml:
       sudo systemctl stop rover-setup-web

  4. Запуск ровера:
       sudo systemctl start rover-motord rover-bringup   # или перезагрузка
     Ходовой владеет демон rover-motord (журнал: journalctl -u
     rover-motord -f). Веб-интерфейс: http://<ip-ровера>:8765,
     аварийный телеоп motord: :8767, rosboard: :8888.

Подробности — в README.md.
============================================================
EOF
