#!/usr/bin/env bash
set -euo pipefail

root="$(cd "$(dirname "$0")/.." && pwd)"
cd "$root"

if [ ! -f build/demos/build.ninja ]; then
    cmake --preset demos
fi
cmake --build --preset demos --target roam_demos_screenshots

exec build/demos/demos/roam_demos_screenshots "$@"
