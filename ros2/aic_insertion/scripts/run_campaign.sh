#!/usr/bin/env bash
# Closed-loop validation campaign: for each seed, restart the sim with a
# randomized layout, run the insertion pipeline to a terminal state, and log
# the outcome plus the ground-truth insertion fraction.
#
#   ./ros2/aic_insertion/scripts/run_campaign.sh "1 2 3 4 5" results.csv
#
# Wall time is roughly 4-5 minutes per seed (sim boot dominates).
set -u
SEEDS=${1:-"1 2 3 4 5"}
OUT=${2:-campaign_results.csv}
REPO="$(cd "$(dirname "$0")/../../.." && pwd)"
PYSH="$HOME/isaacsim-6.0/_build/linux-x86_64/release/python.sh"
TIMEOUT_S=420

source /opt/ros/jazzy/setup.bash
source "$REPO/ros2/install/setup.bash"
echo "seed,outcome,retries,fraction,seconds" >> "$OUT"

for SEED in $SEEDS; do
  echo "=== seed $SEED ==="
  pkill -f run_sim.py 2>/dev/null; pkill -f "perception_node|insertion_node" 2>/dev/null
  sleep 5
  "$PYSH" "$REPO/run_sim.py" --headless --camera-res 448 --randomize-seed "$SEED" \
      > "/tmp/campaign_sim_$SEED.log" 2>&1 &
  SIM_PID=$!
  until ros2 topic list 2>/dev/null | grep -q /aic/center_camera/rgb; do
    sleep 3
    kill -0 $SIM_PID 2>/dev/null || { echo "seed $SEED: sim died"; break; }
  done
  sleep 5
  ros2 launch aic_insertion insertion.launch.py > "/tmp/campaign_nodes_$SEED.log" 2>&1 &
  LAUNCH_PID=$!

  START=$(date +%s)
  OUTCOME=TIMEOUT; RETRIES=-1
  while [ $(( $(date +%s) - START )) -lt $TIMEOUT_S ]; do
    S=$(timeout 5 ros2 topic echo /aic/insertion/status --once 2>/dev/null | head -1)
    case "$S" in
      *SEATED*) OUTCOME=SEATED; RETRIES=${S##*retries=}; break;;
      *FAILED*) OUTCOME=FAILED; RETRIES=${S##*retries=}; break;;
    esac
    sleep 5
  done
  ELAPSED=$(( $(date +%s) - START ))
  FRACTION=$(timeout 5 ros2 topic echo /aic/cheat/insertion_fraction --once 2>/dev/null \
      | head -1 | sed 's/data: //')
  echo "seed $SEED -> $OUTCOME retries=$RETRIES fraction=$FRACTION (${ELAPSED}s)"
  echo "$SEED,$OUTCOME,$RETRIES,$FRACTION,$ELAPSED" >> "$OUT"
  kill $LAUNCH_PID 2>/dev/null
  pkill -f "perception_node|insertion_node" 2>/dev/null
  kill $SIM_PID 2>/dev/null
  sleep 5
done
pkill -f run_sim.py 2>/dev/null
echo "campaign done -> $OUT"
