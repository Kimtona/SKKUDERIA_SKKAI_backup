#!/bin/bash
# Apply the simulator submodule patches. Safe to re-run: skips a submodule that
# already carries its patch. See README.md for what these do.
set -e

cd "$(dirname "${BASH_SOURCE[0]}")/.."

apply_one() {
    local path="$1" patch="$2" marker="$3"
    if ! [ -d "$path/.git" ] && ! [ -f "$path/.git" ]; then
        echo "SKIP $path -- submodule not initialised (run: git submodule update --init)"
        return
    fi
    if git -C "$path" log --oneline -1 2>/dev/null | grep -q "$marker"; then
        echo "OK   $path -- already applied"
        return
    fi
    if ! git -C "$path" diff --quiet || ! git -C "$path" diff --cached --quiet; then
        echo "SKIP $path -- has uncommitted changes, refusing to touch it"
        return
    fi
    git -C "$path" checkout -B skku/reactive-map >/dev/null 2>&1
    git -C "$path" am "$(pwd)/.sim_patches/$patch"
    echo "DONE $path"
}

apply_one base_system/f110_simulator/f1tenth_gym \
          f1tenth_gym__reactive-map.patch "update_map_from_occupancy_grid"
apply_one base_system/f110_simulator/f1tenth_gym_ros \
          f1tenth_gym_ros__reactive-map.patch "reactive /map"

echo
echo "Rebuild so the changes take effect:"
echo "  colcon build --packages-select f1tenth_gym_ros --base-paths ~/ws --symlink-install"
echo "  (f110_gym is pip-installed editable, so its half is already live)"
