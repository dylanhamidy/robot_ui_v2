#!/usr/bin/env bash
set -e
cd "$(dirname "$0")/.."
VERSION=${1:-"1.0.0"}
IMAGE="luxolis/robot_ui:${VERSION}"

# Ensure API-DRFL submodule is checked out (Dockerfile copies lux_drfl_daemon/ into build context)
git submodule update --init lux_drfl_daemon/third_party/API-DRFL

# Minify JS + HTML
bash build/minify.sh

# Build image
docker build --tag "${IMAGE}" --tag "luxolis/robot_ui:latest" .

echo "✓ ${IMAGE}"
