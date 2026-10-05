#!/usr/bin/env bash
# Show live FK positions and rotations of both arm end links, in base_link coordinates.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/env.sh"
exec /usr/bin/python3 "$ROOT/sim_ws/src/g2_control/test/test_left_arm_1mm.py" watch
