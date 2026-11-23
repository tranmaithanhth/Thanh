#!/usr/bin/env bash
# Spawn ONLY stage 3 (Ackermann drive, no LiDAR and no depth camera).
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$ROOT/spawn_robot_stage.sh" 3
