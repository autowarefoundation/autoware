#!/usr/bin/env bash
# Apply the lab patches to upstream repos that have no rancslab fork. Run after `vcs import`, before
# `colcon build`. Safe to re-run: a patch that is already applied is skipped.
#
#   patches/autoware_core.patch   EKF diagnostics fix (2026-10-15: gates reset every 50 Hz cycle -> fake
#                                 "pose topic is delay" WARN -> MRM emergency stop flicker) and
#                                 route_handler yaw_threshold pi/2 -> pi
set -eo pipefail
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
SRC=$HERE/../src

apply() {  # apply <repo dir under src> <patch file>
  local repo=$SRC/$1 patch=$HERE/patches/$2
  if git -C "$repo" apply --reverse --check "$patch" 2>/dev/null; then echo "ok  $2 (already applied)"
  elif git -C "$repo" apply --check "$patch"; then git -C "$repo" apply "$patch"; echo "ok  $2 applied to src/$1"
  else echo "!! $2 does not apply to src/$1 (version changed?)"; exit 1; fi
}

apply core/autoware_core autoware_core.patch
