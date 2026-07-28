#!/usr/bin/env bash
# Установка systemd-юнита rover-can-teleop (запасной телеоп без ROS).
# УСТАРЕВАЕТ: штатно ходовой владеет демон rover-motord (tools/motord) со
# своим аварийным телеопом на :8767 — этот юнит остаётся последним резервом.
# Запускать на ровере из каталога tools/can_teleop (или откуда угодно):
#   bash ~/rover_ws/tools/can_teleop/install.sh
# Юнит ставится ВЫКЛЮЧЕННЫМ (это фолбэк): включается вручную, при старте
# сам останавливает rover-motord/rover-bringup/rover-setup-web (Conflicts=).
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
SERVICE_NAME="rover-can-teleop.service"

# Без захардкоженного пользователя: по умолчанию — вызывающий (SUDO_USER при sudo).
RUN_USER="${ROVER_SERVICE_USER:-${SUDO_USER:-$(id -un)}}"
RUN_GROUP="${ROVER_SERVICE_GROUP:-${RUN_USER}}"

PYTHON_BIN="$(command -v python3)"

echo "Проверка зависимостей Python (${PYTHON_BIN})..."
if ! "${PYTHON_BIN}" -c 'import can, serial, yaml' 2>/dev/null; then
  cat >&2 <<EOF
ВНИМАНИЕ: не хватает python-модулей (нужны python-can, pyserial, PyYAML).
На ровере они уже стоят для ROS-стека; если нет:
  sudo apt install python3-can python3-serial python3-yaml
  # или: pip3 install --break-system-packages python-can pyserial pyyaml
Продолжаю установку юнита.
EOF
fi

tmp="$(mktemp)"
sed \
  -e "s|^User=.*|User=${RUN_USER}|" \
  -e "s|^Group=.*|Group=${RUN_GROUP}|" \
  -e "s|^WorkingDirectory=.*|WorkingDirectory=${SCRIPT_DIR}|" \
  -e "s|^ExecStart=.*|ExecStart=${PYTHON_BIN} ${SCRIPT_DIR}/can_teleop.py|" \
  "${SCRIPT_DIR}/${SERVICE_NAME}" > "${tmp}"
sudo install -m 0644 "${tmp}" "/etc/systemd/system/${SERVICE_NAME}"
rm -f "${tmp}"

sudo systemctl daemon-reload
# НЕ включаем автозапуск: это запасной канал. Автостарт по-прежнему у
# rover-bringup; переключение — командой start (Conflicts= разрулит сам).

cat <<EOF

Установлено: ${SERVICE_NAME} (выключен, пользователь ${RUN_USER})

Переключиться на запасной телеоп (ROS остановится сам):
  sudo systemctl start ${SERVICE_NAME}
  # телефон: http://10.42.0.1:8765

Вернуться на ROS-стек (телеоп остановится сам):
  sudo systemctl start rover-bringup

Сделать запасной телеоп автозапуском вместо ROS:
  sudo systemctl disable rover-bringup
  sudo systemctl enable ${SERVICE_NAME}

Журнал:
  journalctl -u ${SERVICE_NAME} -f
EOF
