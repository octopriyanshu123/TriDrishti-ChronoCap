#!/usr/bin/env bash

set -euo pipefail

source "$(cd "$(dirname "$0")" && pwd)/common.sh"

BUILD_TYPE="${BUILD_TYPE:-Debug}"
I2W_ECAL_INSTALL_DIR="${I2W_ECAL_INSTALL_DIR:-${PROJECT_ROOT}/../i2w/build/third_party/ecal-install}"

find_local_cmake_bin() {
  local sdk_root="$1"
  if [[ ! -d "${sdk_root}" ]]; then
    return 0
  fi

  local arch cmake_dir
  arch="$(uname -m)"
  cmake_dir="$(find "${sdk_root}" -maxdepth 1 -type d -name "cmake-*-linux-${arch}" 2>/dev/null | sort -V | tail -n 1)"
  if [[ -n "${cmake_dir}" ]]; then
    printf '%s/bin\n' "${cmake_dir}"
  fi
}

LOCAL_CMAKE_BIN="$(find_local_cmake_bin "${PROJECT_ROOT}/../../sdks")"
if [[ -n "${LOCAL_CMAKE_BIN}" && -x "${LOCAL_CMAKE_BIN}/cmake" ]]; then
  export PATH="${LOCAL_CMAKE_BIN}:${PATH}"
fi

require_cmd() {
  if ! command -v "$1" >/dev/null 2>&1; then
    echo "missing required tool: $1" >&2
    exit 1
  fi
}

jobs() {
  if command -v nproc >/dev/null 2>&1; then
    nproc
    return
  fi
  getconf _NPROCESSORS_ONLN
}

require_cmd cmake
require_cmd c++

prefix_path="${I2W_ECAL_INSTALL_DIR}"
if [[ -n "${CMAKE_PREFIX_PATH:-}" ]]; then
  prefix_path="${I2W_ECAL_INSTALL_DIR};${CMAKE_PREFIX_PATH}"
fi

cmake -S "${PROJECT_ROOT}" -B "${MYACTUATOR_CPF_BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}" \
  -DCMAKE_PREFIX_PATH="${prefix_path}" \
  -DI2W_ECAL_DIR="${I2W_ECAL_INSTALL_DIR}"

cmake --build "${MYACTUATOR_CPF_BUILD_DIR}" -j"$(jobs)"
