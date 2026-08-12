#!/bin/bash

sector_tuner_dir="$(ros2 pkg prefix sector_tuner)"
install_dir="$(dirname "$sector_tuner_dir")"
ws_dir="$(dirname "$install_dir")"
# Move to ws directory or abort if directory doesn't exist
cd "$ws_dir" || exit

# --symlink-install matters here. The workspace is normally built with it, and a
# plain build replaces the installed symlinks with real copies -- after which
# every later edit to a config or map under stack_master is silently ignored,
# because the running stack reads the stale copy in install/. Passing the flag
# restores the symlinks, so this is safe to run on a workspace built either way.
colcon build --packages-select stack_master --symlink-install
