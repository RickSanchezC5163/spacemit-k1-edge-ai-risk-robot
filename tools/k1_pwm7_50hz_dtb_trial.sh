#!/usr/bin/env bash
set -euo pipefail

REPO_DIR="${REPO_DIR:-/home/soc/edge-ai-robot-k1}"
BOOT_DTB="${K1_BOOT_DTB:-/boot/spacemit/6.6.63/k1-x_MUSE-Pi-Pro.dtb}"
EXPECTED_BOOT_DTB_SHA256="${K1_EXPECTED_BOOT_DTB_SHA256:-f755c6ace1367180330dd59d0f7c7a5f984a6c42cb8f2a14e07b2e788ce410ae}"
BACKUP_DTB="${BOOT_DTB}.pre-k1-pwm7-50hz"
OVERLAY_DTS="${REPO_DIR}/configs/k1_pwm7_50hz_overlay.dts"
OUTPUT_DIR="${REPO_DIR}/outputs/k1_pwm7_50hz_trial"
CANDIDATE_DTB="${OUTPUT_DIR}/k1-x_MUSE-Pi-Pro-pwm7-50hz.dtb"
OVERLAY_DTBO="${OUTPUT_DIR}/k1_pwm7_50hz_overlay.dtbo"
DTC="${K1_DTC:-/usr/src/linux-headers-6.6.63/scripts/dtc/dtc}"
FDTOVERLAY="${K1_FDTOVERLAY:-/usr/src/linux-headers-6.6.63/scripts/dtc/fdtoverlay}"
ACTION="${1:-status}"

require_root() {
  [[ "$(id -u)" -eq 0 ]] || { echo "this action requires root; run with sudo" >&2; exit 2; }
}

verify_tools() {
  for path in "${DTC}" "${FDTOVERLAY}" "${OVERLAY_DTS}" "${BOOT_DTB}"; do
    [[ -e "${path}" ]] || { echo "required path is missing: ${path}" >&2; exit 3; }
  done
}

show_pwm7_node() {
  "${DTC}" -I dtb -O dts "$1" 2>/dev/null | awk '
    /pwm@d401bc00 \{/ { printing=1 }
    printing { print }
    printing && /^\t\t};$/ { exit }
  '
}

build_candidate() {
  verify_tools
  mkdir -p "${OUTPUT_DIR}"
  local base_dtb="${BOOT_DTB}"
  [[ ! -f "${BACKUP_DTB}" ]] || base_dtb="${BACKUP_DTB}"
  local base_sha
  base_sha="$(sha256sum "${base_dtb}" | awk '{print $1}')"
  if [[ -z "${EXPECTED_BOOT_DTB_SHA256}" || "${base_sha}" != "${EXPECTED_BOOT_DTB_SHA256}" ]]; then
    echo "refusing to build from an unexpected base DTB: ${base_sha}" >&2
    echo "set K1_EXPECTED_BOOT_DTB_SHA256 only after independently verifying this board image" >&2
    exit 4
  fi
  "${DTC}" -@ -I dts -O dtb -o "${OVERLAY_DTBO}" "${OVERLAY_DTS}"
  "${FDTOVERLAY}" -i "${base_dtb}" -o "${CANDIDATE_DTB}" "${OVERLAY_DTBO}"
  local node
  node="$(show_pwm7_node "${CANDIDATE_DTB}")"
  grep -q 'assigned-clock-rates = <0x8000>;' <<<"${node}"
  grep -q 'pinctrl-0 = <0x' <<<"${node}"
  grep -q 'status = "okay";' <<<"${node}"
  echo "candidate built: ${CANDIDATE_DTB}"
  sha256sum "${base_dtb}" "${CANDIDATE_DTB}"
}

case "${ACTION}" in
  build)
    build_candidate
    ;;
  status)
    verify_tools
    echo "boot DTB: ${BOOT_DTB}"
    sha256sum "${BOOT_DTB}"
    show_pwm7_node "${BOOT_DTB}"
    ;;
  install)
    require_root
    build_candidate
    current_sha="$(sha256sum "${BOOT_DTB}" | awk '{print $1}')"
    [[ "${current_sha}" == "${EXPECTED_BOOT_DTB_SHA256}" ]] || {
      echo "refusing to replace an unexpected live DTB: ${current_sha}" >&2
      exit 5
    }
    if [[ -e "${BACKUP_DTB}" ]]; then
      backup_sha="$(sha256sum "${BACKUP_DTB}" | awk '{print $1}')"
      [[ "${backup_sha}" == "${EXPECTED_BOOT_DTB_SHA256}" ]] || {
        echo "existing backup has an unexpected hash: ${backup_sha}" >&2
        exit 6
      }
    else
      cp --preserve=mode,ownership,timestamps "${BOOT_DTB}" "${BACKUP_DTB}"
    fi
    install -m 0644 "${CANDIDATE_DTB}" "${BOOT_DTB}.new"
    sync "${BOOT_DTB}.new" "${BACKUP_DTB}"
    mv "${BOOT_DTB}.new" "${BOOT_DTB}"
    sync "${BOOT_DTB}"
    echo "PWM7 50 Hz candidate installed. Reboot is required."
    ;;
  rollback)
    require_root
    [[ -f "${BACKUP_DTB}" ]] || { echo "backup is missing: ${BACKUP_DTB}" >&2; exit 7; }
    install -m 0644 "${BACKUP_DTB}" "${BOOT_DTB}.rollback"
    sync "${BOOT_DTB}.rollback"
    mv "${BOOT_DTB}.rollback" "${BOOT_DTB}"
    sync "${BOOT_DTB}"
    echo "original DTB restored. Reboot is required."
    ;;
  *) echo "usage: $0 {build|status|install|rollback}" >&2; exit 1 ;;
esac
