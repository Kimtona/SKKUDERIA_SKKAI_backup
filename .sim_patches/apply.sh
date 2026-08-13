#!/bin/bash
# Apply the simulator submodule patches, in order. Safe to re-run: patches already
# present are skipped. See README.md for what these do.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")/.."
ROOT="$(pwd)"

GYM=base_system/f110_simulator/f1tenth_gym
GYM_ROS=base_system/f110_simulator/f1tenth_gym_ros

# "<submodule path>|<patch file>|<subject fragment proving it is applied>"
#
# Order matters: these are `git am` patches, so within one submodule they must be
# listed in the order their commits were made. Each gym_ros patch touches
# gym_bridge.py, and obstacle-size-param edits a block that reactive-map
# introduces, so it has to come last.
PATCHES=(
    "$GYM|f1tenth_gym__reactive-map.patch|update_map_from_occupancy_grid"
    "$GYM|f1tenth_gym__agent-collisions.patch|agent-agent collisions"
    "$GYM_ROS|f1tenth_gym_ros__reactive-map.patch|reactive /map"
    "$GYM_ROS|f1tenth_gym_ros__agent-collisions.patch|agent_collisions as a bridge"
    "$GYM_ROS|f1tenth_gym_ros__agent-collisions-launcharg.patch|agent_collisions as a launch"
    "$GYM_ROS|f1tenth_gym_ros__obstacle-size-param.patch|obstacle_size be set from"
)

for entry in "${PATCHES[@]}"; do
    IFS='|' read -r path patch marker <<< "$entry"

    if ! [ -e "$path/.git" ]; then
        echo "SKIP $patch -- $path not initialised (run: git submodule update --init)"
        continue
    fi
    if git -C "$path" log --oneline -20 2>/dev/null | grep -qF "$marker"; then
        echo "OK   $patch -- already applied"
        continue
    fi
    if ! git -C "$path" diff --quiet || ! git -C "$path" diff --cached --quiet; then
        echo "SKIP $patch -- $path has uncommitted changes, refusing to touch it"
        continue
    fi
    # First patch for this submodule starts the branch; later ones stack onto it.
    if ! git -C "$path" rev-parse --verify --quiet skku/reactive-map >/dev/null; then
        git -C "$path" checkout -B skku/reactive-map >/dev/null 2>&1
    else
        git -C "$path" checkout skku/reactive-map >/dev/null 2>&1
    fi
    git -C "$path" am "$ROOT/.sim_patches/$patch"
    echo "DONE $patch"
done

echo
echo "Rebuild so the changes take effect:"
echo "  colcon build --packages-select f1tenth_gym_ros --base-paths ~/ws --symlink-install"
echo "  (f110_gym is pip-installed editable, so its half is already live)"
