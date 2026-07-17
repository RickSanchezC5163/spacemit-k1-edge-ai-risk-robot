#!/usr/bin/env bash
set -euo pipefail

USER_NAME="${1:-soc}"
SESSION_NAME="${2:-bianbu-lite}"
CONF="/etc/sddm.conf.d/sddm-bianbu.conf"
BACKUP="${CONF}.bak.$(date +%Y%m%d_%H%M%S)"

if [[ "$(id -u)" -ne 0 ]]; then
  echo "Run with sudo: sudo bash tools/configure_k1_sddm_autologin.sh ${USER_NAME} ${SESSION_NAME}" >&2
  exit 1
fi

cp "${CONF}" "${BACKUP}"

python3 - "${CONF}" "${USER_NAME}" "${SESSION_NAME}" <<'PY'
from pathlib import Path
import sys

path = Path(sys.argv[1])
user = sys.argv[2]
session = sys.argv[3]
lines = path.read_text(encoding="utf-8").splitlines()

sections = {}
current = None
for idx, line in enumerate(lines):
    stripped = line.strip()
    if stripped.startswith("[") and stripped.endswith("]"):
        current = stripped[1:-1]
        sections[current] = idx

if "Autologin" not in sections:
    if lines and lines[-1].strip():
        lines.append("")
    lines.append("[Autologin]")
    lines.extend([f"User={user}", f"Session={session}", "Relogin=true"])
else:
    start = sections["Autologin"] + 1
    end = len(lines)
    for idx in range(start, len(lines)):
        stripped = lines[idx].strip()
        if stripped.startswith("[") and stripped.endswith("]"):
            end = idx
            break
    block = lines[start:end]
    wanted = {"User": user, "Session": session, "Relogin": "true"}
    seen = set()
    new_block = []
    for line in block:
        key = line.split("=", 1)[0].strip() if "=" in line else ""
        if key in wanted:
            new_block.append(f"{key}={wanted[key]}")
            seen.add(key)
        else:
            new_block.append(line)
    for key, value in wanted.items():
        if key not in seen:
            new_block.append(f"{key}={value}")
    lines = lines[:start] + new_block + lines[end:]

path.write_text("\n".join(lines) + "\n", encoding="utf-8")
PY

echo "Updated ${CONF}"
echo "Backup: ${BACKUP}"
grep -n . "${CONF}"
