#!/usr/bin/env bash
# Build one width variant of a sketched track, end to end, inside the container.
#
#   ./build_track_variant.sh skku_track_22x8_narrow 1.70 1.15 1.10 1.25
#
# The four numbers are the full track width in metres for each sector, in
# driving order from the start line: main straight, right hairpin, middle
# corridor, left S-bend. The centre line is untouched, so every variant is the
# same track shape at a different width.
#
# Steps: render the map -> build the raceline -> write the sector YAMLs ->
# rebuild stack_master so the launch files see the new map. It prints the spawn
# block for config/SIM/sim.yaml at the end but does not install it, since one
# sim.yaml serves every map. Point the simulator at this one with
#
#   python3 map_sectors.py <name> --spawn-only --set-spawn
#
# before launching, or the cars spawn on the previous map's raceline, sit in a
# wall, and f110_gym latches a collision that never clears.
#
# SKETCH_ARGS below traces skku_track_22x8's drawing. Point it at another
# sketch and its own --seed/--box/--start to vary a different track.
set -euo pipefail

NAME=${1:?usage: build_track_variant.sh <name> <w0> <w1> <w2> <w3>}
shift
WIDTHS=("$@")
[ ${#WIDTHS[@]} -eq 4 ] || { echo "need four sector widths, got ${#WIDTHS[@]}" >&2; exit 1; }

SCRIPTS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MAPS=$SCRIPTS/../maps
SKETCH_ARGS=(--seed 400 420 --box 29 842 148 445 --box-size 20 8
             --canvas 22 8 --start 615 --heading 0.0)
SAFETY_WIDTH=${SAFETY_WIDTH:-0.7}

# global_planner's get_data_path() resolves to <ws>/src/stack_master, while the
# repo is mounted at <ws>/src/race_stack. It reads and writes the map through
# that path, so the symlink is not optional.
ln -sfn ~/ws/src/race_stack/stack_master ~/ws/src/stack_master
set +u    # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES before setting it
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
set -u

echo "=== 1/4 render $NAME  widths ${WIDTHS[*]} m"
python3 "$SCRIPTS/sketch_to_map.py" "$MAPS/skku_track_22x8/sketch.png" "$NAME" \
    "${SKETCH_ARGS[@]}" --width "${WIDTHS[@]}" --expect-walls 1 --min-wall 0.15

echo "=== 2/4 raceline (safety_width $SAFETY_WIDTH)"
# plan() writes global_waypoints.json and then the node just spins, so it is
# started in the background and killed once the file has stopped growing.
WPNTS=$MAPS/$NAME/global_waypoints.json
rm -f "$WPNTS"
# Three things stand between here and a raceline: the node asks for TkAgg at
# import time, which fails headless; load_and_plot_map() plots unconditionally
# and blocks on an interactive backend; and global_planner_params.yaml keys a
# node name the node does not use, so every parameter goes in with -p.
MPLBACKEND=Agg python3 -c "
import matplotlib; matplotlib.use('Agg'); matplotlib.use = lambda *a, **k: None
import matplotlib.pyplot as plt; plt.show = lambda *a, **k: None
from global_planner.global_planner_node import main; main()" \
    --ros-args -p map_name:="$NAME" -p safety_width:="$SAFETY_WIDTH" \
               -p safety_width_sp:="$SAFETY_WIDTH" -p reverse_mapping:=False \
               -p show_plots:=False &
PLANNER=$!
SIZE=0
for _ in $(seq 120); do
    sleep 1
    kill -0 $PLANNER 2>/dev/null || break
    NOW=$(stat -c %s "$WPNTS" 2>/dev/null || echo 0)
    [ "$NOW" != 0 ] && [ "$NOW" = "$SIZE" ] && break
    SIZE=$NOW
done
kill $PLANNER 2>/dev/null || true
wait $PLANNER 2>/dev/null || true
[ -s "$WPNTS" ] || { echo "no raceline written -- see the planner output above" >&2; exit 1; }

echo "=== 3/4 sectors"
python3 "$SCRIPTS/map_sectors.py" "$NAME"

echo "=== 4/4 build"
cd ~/ws && colcon build --packages-select stack_master --symlink-install >/dev/null
cat <<EOM
done. To drive it:
  python3 $SCRIPTS/map_sectors.py $NAME --spawn-only --set-spawn
  ros2 launch stack_master base_system_launch.xml sim:=True map_name:=$NAME racecar_version:=SIM
  ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM LU_table:=SIM_linear ctrl_algo:=PP
EOM
