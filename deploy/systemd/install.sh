#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd -- "${SCRIPT_DIR}/../.." && pwd)"

MOTORD_SERVICE="rover-motord.service"
BRINGUP_SERVICE="rover-bringup.service"
SETUP_WEB_SERVICE="rover-setup-web.service"
ENV_NAME="rover-bringup"
ENV_SRC="${SCRIPT_DIR}/${ENV_NAME}.env"
ENV_DST="/etc/default/${ENV_NAME}"

# No hardcoded user: default to the invoking user (SUDO_USER when run via sudo).
RUN_USER="${ROVER_SERVICE_USER:-${SUDO_USER:-$(id -un)}}"
RUN_GROUP="${ROVER_SERVICE_GROUP:-${RUN_USER}}"
WORKSPACE="${ROVER_WS:-${REPO_ROOT}}"
RUN_HOME="$(getent passwd "${RUN_USER}" | cut -d: -f6)"
CONFIG_DIR="${ROVER_CONFIG_DIR:-${RUN_HOME}/rover_config}"

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  cat >&2 <<EOF
Workspace install/setup.bash was not found:
  ${WORKSPACE}/install/setup.bash

Build the workspace first, or pass the workspace path explicitly:
  ROVER_WS=/home/${RUN_USER}/rover_ws bash ${SCRIPT_DIR}/install.sh
EOF
  exit 1
fi

install_service() {
  local name="$1"
  local tmp
  tmp="$(mktemp)"
  # Глобальная замена пути workspace покрывает и WorkingDirectory, и
  # ExecStart (у rover-motord путь входит в командную строку).
  sed \
    -e "s|^User=.*|User=${RUN_USER}|" \
    -e "s|^Group=.*|Group=${RUN_GROUP}|" \
    -e "s|/home/ubuntu/rover_ws|${WORKSPACE}|g" \
    "${SCRIPT_DIR}/${name}" > "${tmp}"
  sudo install -m 0644 "${tmp}" "/etc/systemd/system/${name}"
  rm -f "${tmp}"
}

install_service "${MOTORD_SERVICE}"
install_service "${BRINGUP_SERVICE}"
install_service "${SETUP_WEB_SERVICE}"

if [[ ! -f "${ENV_DST}" ]]; then
  tmp_env="$(mktemp)"
  sed \
    -e "s|^ROVER_WS=.*|ROVER_WS=${WORKSPACE}|" \
    -e "s|^ROVER_CONFIG_DIR=.*|ROVER_CONFIG_DIR=${CONFIG_DIR}|" \
    "${ENV_SRC}" > "${tmp_env}"
  sudo install -m 0640 -o root -g "${RUN_GROUP}" "${tmp_env}" "${ENV_DST}"
  rm -f "${tmp_env}"
else
  echo "Keeping existing ${ENV_DST}"
fi

sudo systemctl daemon-reload
sudo systemctl enable "${MOTORD_SERVICE}"
sudo systemctl enable "${BRINGUP_SERVICE}"
# rover-setup-web is intentionally NOT enabled: it owns the same USB-CAN
# adapter as rover-motord, so it is started manually for first-run setup only.

cat <<EOF
Installed:
  ${MOTORD_SERVICE}     (enabled — motor daemon, owns the USB-CAN adapter;
                         emergency phone teleop on :8767, user ${RUN_USER})
  ${BRINGUP_SERVICE}    (enabled, autostart on boot; VESC node is a bridge
                         to ${MOTORD_SERVICE})
  ${SETUP_WEB_SERVICE}  (installed, disabled — start manually)

Edit launch settings (MOTORD_OPTS lives here too):
  sudo nano ${ENV_DST}

Control:
  sudo systemctl start ${MOTORD_SERVICE} ${BRINGUP_SERVICE}
  systemctl status ${MOTORD_SERVICE}
  journalctl -u ${MOTORD_SERVICE} -f
  journalctl -u ${BRINGUP_SERVICE} -f

First-run VESC setup (setup-web stops motord+bringup itself via Conflicts=):
  sudo systemctl start ${SETUP_WEB_SERVICE}
  # The web app binds 127.0.0.1:5000 on the rover; open an SSH tunnel:
  #   ssh -L 5000:127.0.0.1:5000 ${RUN_USER}@<rover>
  # then browse http://127.0.0.1:5000, export motors.yaml and:
  sudo systemctl stop ${SETUP_WEB_SERVICE}
  sudo systemctl start ${MOTORD_SERVICE} ${BRINGUP_SERVICE}
EOF
