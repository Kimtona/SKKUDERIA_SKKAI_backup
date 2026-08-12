# Preserved simulator work — reference only

Snapshot of simulation-enabling work that existed **only as uncommitted changes
inside the `f1tenth_gym` / `f1tenth_gym_ros` submodules** on one MacBook. Those
submodules point at ForzaETH upstream, which we cannot push to, so the commits
are exported here as patch files instead.

Captured 2026-08-13. **Nothing here is verified to run.**

## Contents

| File | Applies to submodule | Base commit |
|---|---|---|
| `f1tenth_gym__reactive-map.patch` | `base_system/f110_simulator/f1tenth_gym` | `f8f8c9d` |
| `f1tenth_gym_ros__reactive-map.patch` | `base_system/f110_simulator/f1tenth_gym_ros` | `6ffd209` |

Both base commits match the submodule SHAs recorded on `fresh`, so they apply
onto a freshly initialised checkout:

```
git -C <submodule path> am < .preserved_sim_patches/<file>.patch
```

## What the patches do

**`f1tenth_gym`** — adds `ScanSimulator2D.update_map_from_occupancy_grid()`, so
the scan simulator's bitmap can be refreshed at runtime from a live
`nav_msgs/OccupancyGrid` rather than only from a map image at `gym.make()` time.
Self-contained; nothing calls it without the second patch.

**`f1tenth_gym_ros`** — subscribes and re-publishes `/map`, paints and erases
obstacles from `/clicked_point`, and adds an InteractiveMarker "Clear Obstacles"
button modelled on the ROS1 `f1tenth_simulator`'s `node/simulator.cpp`.

## Read this before using the gym_bridge patch

**It is known broken. Do not cherry-pick it blindly.**

It is the remnant of a session whose changes were supposed to be fully reverted
— parent commit `7f2b353` states "No trace of that work remains in the tree".
That is inaccurate: `package.xml` was reverted but `gym_bridge.py` was not. The
file imports `interactive_markers` while neither `package.xml` nor
`.install_utils/linux_req/linux_req.txt` provides it, so the node dies at
startup, which cascades into `detect` TF failures (`"map"` -> `""`).

Likely repair: add `ros-humble-interactive-markers` to `linux_req.txt` and an
`exec_depend` to the submodule's `package.xml`. Unverified. If it still fails,
the map-painting core (`map_callback` + the `f1tenth_gym` patch) works
independently of the button UI and can be kept on its own.

## Related preserved work

Tag **`sim-restore-patch`** (`40e0feb`) holds the parent-repo half: 8 files
covering `force_all_ot_true`, the `opponent_publisher` virtual/lidar default,
and sim tuning values. Same caveat — reference material, not a patch to apply.
Note especially that it writes sim-only tuning into *shared* config
(`lateral_width_gb_m` 1.1 -> 0.2, `obs_traj_tresh` 0.3 -> 1.0); those belong in
a `SIM/` profile, not in files the real car reads.
