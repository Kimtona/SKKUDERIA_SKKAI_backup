#!/usr/bin/env bash
# Bring up any map in the simulator, in one command.
#
#   ./sim_map.sh skku_track_22x8_narrow                  # time trials
#   ./sim_map.sh skku_track_22x8_wide head_to_head       # overtake stack
#
# Ctrl-C stops everything it started. It does the four things that fail
# silently if you launch by hand: starts the Zenoh router, puts the map's own
# spawn poses in config/SIM/sim.yaml, passes map_name to both launch files, and
# kicks /opp_drive when sim.yaml asks for two cars.
#
# It refuses to start on top of a running stack. `pkill -f "ros2 launch"` leaves
# the child nodes alive, so a second launch coexists with the first: two bridges
# stepping two maps, both publishing /drive and /car_state/odom, and the result
# is a map that looks like it crashes the car when it does not. The message says
# what to kill.
set -euo pipefail

MAP=${1:?usage: sim_map.sh <map_name> [time_trials|head_to_head]}
MODE=${2:-time_trials}
case $MODE in time_trials|head_to_head) ;; *) echo "unknown mode $MODE" >&2; exit 1 ;; esac

SCRIPTS=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MAPDIR=$SCRIPTS/../maps/$MAP
SIM_YAML=$SCRIPTS/../config/SIM/sim.yaml
LU_TABLE=${LU_TABLE:-SIM_linear}
CTRL=${CTRL_ALGO:-PP}          # PP is the default controller; the launch files
                               # still say MAP, which is stale.

for f in "$MAP.yaml" "$MAP.png" global_waypoints.json ot_sectors.yaml speed_scaling.yaml; do
    [ -f "$MAPDIR/$f" ] || { echo "$MAPDIR/$f is missing -- is $MAP a built map?" >&2; exit 1; }
done

BUSY=$(ps -eo stat,cmd | grep -E "gym_bridge|ws/install" | grep -v grep \
       | awk '$1 !~ /Z/' | wc -l)     # <defunct> bridges never get reaped here
if [ "$BUSY" -gt 0 ]; then
    echo "$BUSY ROS processes are already running. Stop them first:" >&2
    echo '  for p in "ros2 launch" "ws/install" "/opt/ros/humble/lib" rviz2 gym_bridge; do' >&2
    echo '      pgrep -af "$p" | grep -v zenohd | awk "{print \$1}" | xargs -r kill -9; done' >&2
    exit 1
fi

set +u    # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES before setting it
source /opt/ros/humble/setup.bash
source ~/ws/install/setup.bash
set -u

# rmw_zenoh needs a router or the nodes never discover each other, and the only
# symptom is lifecycle_manager waiting on map_server forever. It has to outlive
# the stack, so it is left running on exit.
if ! pgrep -f rmw_zenohd > /dev/null; then
    echo "== starting the Zenoh router"
    # setsid, not just &: the exit trap below kills this script's whole process
    # group, and the router has to survive that.
    setsid ros2 run rmw_zenoh_cpp rmw_zenohd > /tmp/rmw_zenohd.log 2>&1 &
    sleep 3
fi

# The bridge spawns the cars at raw map coordinates whatever map_name says, and
# a car that starts inside a wall latches a collision it never leaves.
echo "== spawn poses from $MAP's raceline"
python3 "$SCRIPTS/map_sectors.py" "$MAP" --spawn-only --set-spawn | sed -n '/ACTIVE MAP/,$p'

AGENTS=$(awk '/^[[:space:]]*num_agent:/ {print $2}' "$SIM_YAML")
if [ "$MODE" = head_to_head ] && [ "$AGENTS" != 2 ]; then
    echo "note: num_agent is $AGENTS in sim.yaml, so there is no opponent car to overtake." >&2
fi

# Untrap before killing: the handler signals its own process group, so a bare
# `trap "kill 0" TERM` re-enters itself until something in the group dies.
cleanup() { trap - EXIT INT TERM; kill 0; }
trap cleanup EXIT INT TERM

echo "== base system on $MAP"
BASE_LOG=/tmp/$MAP.base.log
: > "$BASE_LOG"
ros2 launch stack_master base_system_launch.xml \
    sim:=True map_name:="$MAP" racecar_version:=SIM > "$BASE_LOG" 2>&1 &

# No ros2 CLI until the launch has started its nodes. Both ends want the ros2
# daemon, and running `ros2 topic echo` while the launch is still evaluating
# deadlocks it: the launch sits on its second line of output and never starts a
# single node. Wait on the log, which costs nothing, and only then ask a topic.
for _ in $(seq 60); do
    grep -q 'process started with pid' "$BASE_LOG" && break
    sleep 1
done
grep -q 'process started with pid' "$BASE_LOG" \
    || { echo "the launch started nothing -- see $BASE_LOG" >&2; exit 1; }
if ! timeout 90 ros2 topic echo /car_state/odom --once > /dev/null 2>&1; then
    echo "the bridge never published /car_state/odom -- see $BASE_LOG" >&2
    exit 1
fi

echo "== $MODE"
MODE_LOG=/tmp/$MAP.$MODE.log
: > "$MODE_LOG"
if [ "$MODE" = time_trials ]; then
    ros2 launch stack_master time_trials_launch.xml racecar_version:=SIM \
        LU_table:="$LU_TABLE" ctrl_algo:="$CTRL" > "$MODE_LOG" 2>&1 &
else
    ros2 launch stack_master head_to_head_launch.xml racecar_version:=SIM \
        LU_table:="$LU_TABLE" map_name:="$MAP" sim:=True ctrl_algo:="$CTRL" \
        > "$MODE_LOG" 2>&1 &
fi
for _ in $(seq 60); do                    # again, no CLI while it evaluates
    grep -q 'process started with pid' "$MODE_LOG" && break
    sleep 1
done

# With num_agent > 1 the bridge refuses to step until BOTH cars have had a drive
# command, so an unkicked /opp_drive freezes the ego as well.
if [ "$AGENTS" != 1 ]; then
    sleep 5
    echo "== kicking /opp_drive so the bridge starts stepping"
    ros2 topic pub -1 /opp_drive ackermann_msgs/msg/AckermannDriveStamped \
        '{drive: {speed: 0.0}}' > /dev/null
fi

cat <<EOM

$MAP is up in $MODE. RViz is on the noVNC desktop (http://localhost:20501/vnc.html).
Logs: $BASE_LOG and $MODE_LOG
Ctrl-C here stops the stack.
EOM
wait
