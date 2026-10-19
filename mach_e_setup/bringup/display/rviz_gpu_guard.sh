#!/usr/bin/env bash
# Safety net for the rviz2 GPU leak: if total GPU memory passes LIMIT MiB, kill rviz2 (autoware.launch
# respawns it, ~10 s without a view). On 2026-10-15 a full GPU killed the CUDA lidar preprocessing
# (cudaErrorMemoryAllocation) -> no NDT, no perception. Better to lose the view for 10 s.
#   rviz_gpu_guard.sh [LIMIT_MiB=13000]
LIMIT=${1:-13000}
while sleep 5; do
  used=$(nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | head -1)
  [ -z "$used" ] && continue
  if [ "$used" -gt "$LIMIT" ]; then
    pid=$(pgrep -f "^/opt/ros/humble/lib/rviz2/rviz2 -d" | head -1)
    [ -z "$pid" ] && continue
    echo "$(date +%T) GPU ${used} MiB > ${LIMIT} MiB -- restarting RViz (pid $pid)"
    kill -9 "$pid"
    sleep 20
  fi
done
