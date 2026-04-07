#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BUILD_DIR="${SCRIPT_DIR}/build"

usage() {
  cat <<EOF
Usage: $(basename "$0") [build|clean|rebuild|test]

Commands:
  build    Configure and build the project in ${BUILD_DIR}
  clean    Remove the build directory
  rebuild  Clean then build again
  test     Run ctest in the build directory
EOF
}

if [ $# -eq 0 ]; then
  set -- build
fi

case "$1" in
  build)
    mkdir -p "$BUILD_DIR"
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR"
    cmake --build "$BUILD_DIR"
    ;;

  clean)
    if [ -d "$BUILD_DIR" ]; then
      rm -rf "$BUILD_DIR"
      echo "Removed build directory: $BUILD_DIR"
    else
      echo "Build directory does not exist: $BUILD_DIR"
    fi
    ;;

  rebuild)
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR"
    cmake --build "$BUILD_DIR"
    ;;

  test)
    if [ ! -d "$BUILD_DIR" ]; then
      echo "Build directory not found. Run './$(basename "$0") build' first."
      exit 1
    fi
    # Force the system libstdc++ path so the test executable does not pick up the conda library.
    LD_LIBRARY_PATH=/lib/x86_64-linux-gnu cmake --build "$BUILD_DIR" --target test
    ;;

  *)
    usage
    exit 1
    ;;
 esac
