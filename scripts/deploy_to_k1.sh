#!/usr/bin/env bash
set -Eeuo pipefail

usage() {
  cat <<'EOF'
Usage:
  scripts/deploy_to_k1.sh --user USER [--host IP] [--remote-dir REMOTE_DIR]
  scripts/deploy_to_k1.sh -u USER [-i IP] [-d REMOTE_DIR]

Defaults:
  IP: 192.168.43.40
  REMOTE_DIR: ~/edge-ai-robot-k1

This script only uploads files. It does not start ROS, move the chassis, or run the arm.

Example:
  scripts/deploy_to_k1.sh --user soc --host 192.168.43.40 --remote-dir ~/edge-ai-robot-k1
EOF
}

K1_USER=""
K1_IP="192.168.43.40"
REMOTE_DIR="~/edge-ai-robot-k1"

while [ "$#" -gt 0 ]; do
  case "$1" in
    -u|--user)
      K1_USER="$2"
      shift 2
      ;;
    -i|--host)
      K1_IP="$2"
      shift 2
      ;;
    -d|--remote-dir)
      REMOTE_DIR="$2"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "ERROR: unknown argument: $1" >&2
      usage
      exit 2
      ;;
  esac
done

if [ -z "$K1_USER" ]; then
  echo "ERROR: K1 user is required." >&2
  usage
  exit 2
fi

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TARGET="${K1_USER}@${K1_IP}"

EXCLUDES=(
  --exclude ".git/"
  --exclude "build/"
  --exclude "install/"
  --exclude "log/"
  --exclude "logs/"
  --exclude "__pycache__/"
  --exclude ".pytest_cache/"
  --exclude "datasets/"
  --exclude "models/"
  --exclude "weights/"
  --exclude "*.pyc"
  --exclude "*.bag"
  --exclude "*.db3"
  --exclude "*.mcap"
  --exclude "*.mp4"
  --exclude "*.avi"
  --exclude "*.onnx"
  --exclude "*.engine"
  --exclude "*.pt"
  --exclude "*.pth"
  --exclude "*.zip"
  --exclude "*.7z"
  --exclude "*.rar"
  --exclude "*.tar.gz"
)

ssh "$TARGET" "mkdir -p $REMOTE_DIR"

if command -v rsync >/dev/null 2>&1; then
  rsync -av "${EXCLUDES[@]}" "$ROOT/" "$TARGET:$REMOTE_DIR/"
else
  echo "rsync not found; falling back to scp tar stream."
  tar \
    --exclude=".git" \
    --exclude="build" \
    --exclude="install" \
    --exclude="log" \
    --exclude="logs" \
    --exclude="__pycache__" \
    -C "$ROOT" -cf - . | ssh "$TARGET" "tar -xf - -C $REMOTE_DIR"
fi

echo "Uploaded to $TARGET:$REMOTE_DIR"
echo "This script only uploaded files; it did not start ROS or move the robot."
