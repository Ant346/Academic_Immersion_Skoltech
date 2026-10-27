#!/usr/bin/env sh
set -eu
cd "$(dirname "$0")"
docker build --builder "${CALIBRATION_BUILDER:-default}" -f Dockerfile.web -t ext_cam-calibration:latest .
docker compose up -d --no-build --wait --wait-timeout 60
printf '\nКалибровка камеры: http://localhost:8080\n'
