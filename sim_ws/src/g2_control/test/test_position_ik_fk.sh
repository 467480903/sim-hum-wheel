#!/usr/bin/env bash
# Run the 1 mm IK -> trajectory -> FK check, or pass --trace to inspect browser commands.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$ROOT/env.sh"
MODE="${1:-move}"
case "$MODE" in
  --trace) MODE=trace ;;
  --dry-run) MODE=dry-run ;;
esac
exec /usr/bin/python3 "$ROOT/sim_ws/src/g2_control/test/test_left_arm_1mm.py" "$MODE"
