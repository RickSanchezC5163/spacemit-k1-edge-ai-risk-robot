#!/usr/bin/env bash
set -euo pipefail

if [[ "$(id -u)" -ne 0 ]]; then
  echo "run this installer with sudo" >&2
  exit 2
fi

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
install -m 0755 "${REPO_DIR}/scripts/k1_light_mode.sh" /usr/local/sbin/k1-light-mode
install -m 0755 "${REPO_DIR}/scripts/k1_pwm7_light_init.sh" /usr/local/sbin/k1-pwm7-light-init

cat > /etc/systemd/system/k1-pwm7-light-off.service <<'UNIT'
[Unit]
Description=Initialize K1 PWM7 lamp output in the off state
DefaultDependencies=no
After=systemd-remount-fs.service
Before=sysinit.target multi-user.target

[Service]
Type=oneshot
ExecStart=/usr/local/sbin/k1-pwm7-light-init
ExecStop=/usr/local/sbin/k1-light-mode off
RemainAfterExit=yes

[Install]
WantedBy=sysinit.target
UNIT

cat > /etc/sudoers.d/k1-light-mode <<'SUDOERS'
Cmnd_Alias K1_LIGHT_MODE = /usr/local/sbin/k1-light-mode on, /usr/local/sbin/k1-light-mode off, /usr/local/sbin/k1-light-mode status
soc ALL=(root) NOPASSWD: K1_LIGHT_MODE
SUDOERS
chmod 0440 /etc/sudoers.d/k1-light-mode
visudo -cf /etc/sudoers.d/k1-light-mode

systemctl disable --now k1-gpio37-light-off.service 2>/dev/null || true
systemctl daemon-reload
systemctl enable --now k1-pwm7-light-off.service
systemctl --no-pager --full status k1-pwm7-light-off.service
