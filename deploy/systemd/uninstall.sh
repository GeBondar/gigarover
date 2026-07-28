#!/usr/bin/env bash
set -euo pipefail

for service in rover-setup-web.service rover-bringup.service rover-motord.service; do
  sudo systemctl disable --now "${service}" 2>/dev/null || true
  sudo rm -f "/etc/systemd/system/${service}"
done
sudo systemctl daemon-reload

cat <<EOF
Removed rover-motord.service, rover-bringup.service and rover-setup-web.service.

The environment file was left in place:
  /etc/default/rover-bringup

Remove it manually if it is no longer needed.
EOF
