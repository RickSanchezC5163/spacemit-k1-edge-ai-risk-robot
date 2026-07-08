#!/usr/bin/env bash
set -Eeuo pipefail

GPIO="${GPIO:-37}"
HELPER="/usr/local/sbin/k1-gpio${GPIO}-light-off.sh"
SERVICE="/etc/systemd/system/k1-gpio${GPIO}-light-off.service"

if [ "$(id -u)" -ne 0 ]; then
  echo "ERROR: run with sudo." >&2
  exit 1
fi

cat > "$HELPER" <<EOF
#!/usr/bin/env sh
set -eu
GPIO="$GPIO"
GPIO_DIR="/sys/class/gpio/gpio\$GPIO"

if [ ! -d "\$GPIO_DIR" ]; then
  echo "\$GPIO" > /sys/class/gpio/export 2>/dev/null || true
  i=0
  while [ ! -d "\$GPIO_DIR" ] && [ "\$i" -lt 20 ]; do
    i=\$((i + 1))
    sleep 0.05
  done
fi

if [ -d "\$GPIO_DIR" ]; then
  echo out > "\$GPIO_DIR/direction"
  echo 0 > "\$GPIO_DIR/value"
  chmod 666 "\$GPIO_DIR/direction" "\$GPIO_DIR/value" 2>/dev/null || true
fi
EOF

chmod 0755 "$HELPER"

cat > "$SERVICE" <<EOF
[Unit]
Description=Force K1 GPIO${GPIO} light control line low at boot
DefaultDependencies=no
After=local-fs.target
Before=basic.target multi-user.target graphical.target

[Service]
Type=oneshot
ExecStart=$HELPER
RemainAfterExit=yes

[Install]
WantedBy=basic.target
EOF

systemctl daemon-reload
systemctl enable "k1-gpio${GPIO}-light-off.service"
systemctl start "k1-gpio${GPIO}-light-off.service"
systemctl status "k1-gpio${GPIO}-light-off.service" --no-pager || true

echo "Installed and started $SERVICE"
echo "GPIO${GPIO} has been forced low. Reboot once to verify boot behavior."
