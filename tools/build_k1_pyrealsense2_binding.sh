#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LIBREALSENSE_SOURCE_DIR="${1:-${LIBREALSENSE_SOURCE_DIR:-}}"
INSTALL_PREFIX="${REALSENSE_PREFIX:-/home/soc/.local/realsense2-2.55.1}"
BUILD_DIR="${REALSENSE_BINDING_BUILD_DIR:-${HOME}/.cache/k1-pyrealsense2-binding}"
PYBIND11_JSON_INCLUDE_DIR="${PYBIND11_JSON_INCLUDE_DIR:-${LIBREALSENSE_SOURCE_DIR}/third-party/pybind11-json/include}"

if [[ -z "${LIBREALSENSE_SOURCE_DIR}" || ! -f "${LIBREALSENSE_SOURCE_DIR}/wrappers/python/pyrealsense2.cpp" ]]; then
  echo "usage: $0 /path/to/librealsense-2.55.1" >&2
  exit 2
fi
if [[ ! -f "${PYBIND11_JSON_INCLUDE_DIR}/pybind11_json/pybind11_json.hpp" ]]; then
  echo "missing pybind11-json header: ${PYBIND11_JSON_INCLUDE_DIR}" >&2
  exit 2
fi

cmake \
  -S "${ROOT_DIR}/tools/realsense_python_binding" \
  -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="${INSTALL_PREFIX}" \
  -DLIBREALSENSE_SOURCE_DIR="${LIBREALSENSE_SOURCE_DIR}" \
  -DPYBIND11_JSON_INCLUDE_DIR="${PYBIND11_JSON_INCLUDE_DIR}" \
  -Dpybind11_DIR=/usr/lib/cmake/pybind11
cmake --build "${BUILD_DIR}" --parallel "${REALSENSE_BUILD_JOBS:-8}"
cmake --install "${BUILD_DIR}"

PYTHONPATH="${INSTALL_PREFIX}/lib/python3.12/site-packages:${PYTHONPATH:-}" \
  python3 -c 'import pyrealsense2 as rs; print("pyrealsense2", getattr(rs, "__version__", "import-ok"))'
