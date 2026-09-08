#!/usr/bin/env bash

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MYACTUATOR_CPF_BUILD_DIR="${MYACTUATOR_CPF_BUILD_DIR:-${PROJECT_ROOT}/build}"

binary_path() {
  printf '%s/%s\n' "${MYACTUATOR_CPF_BUILD_DIR}" "$1"
}

require_binary() {
  local path
  path="$(binary_path "$1")"
  if [[ ! -x "${path}" ]]; then
    echo "missing built binary: ${path}" >&2
    echo "run ./scripts/build.sh first" >&2
    exit 1
  fi
}
