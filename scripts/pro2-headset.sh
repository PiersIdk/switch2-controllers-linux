#!/usr/bin/env bash
# Switch the Pro Controller 2 between headset mode (the 3.5 mm headset's mic
# works, motion doesn't) and default mode (motion works, no mic), live:
#   scripts/pro2-headset.sh [on|off|toggle|status]   (default: toggle)
cd "$(dirname "$(readlink -f "$0")")/.." || exit 1
exec .venv312/bin/python -m ngc headset "${1:-toggle}"
