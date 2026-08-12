# Simulator submodule patches

`f1tenth_gym` and `f1tenth_gym_ros` point at ForzaETH upstream, which we cannot
push to, so changes to them live here as patch files and are applied after
`git submodule update --init`.

Both apply onto the submodule SHAs recorded on this branch (`f8f8c9d` and
`6ffd209`), so a fresh checkout takes them cleanly:

```bash
./.sim_patches/apply.sh
```

## What they add

Placing a static obstacle by clicking **Publish Point** in RViz.

| Patch | Adds |
|---|---|
| `f1tenth_gym__reactive-map.patch` | `ScanSimulator2D.update_map_from_occupancy_grid()` |
| `f1tenth_gym_ros__reactive-map.patch` | `/map` subscribe + re-publish, `/clicked_point` obstacle toggle, `/clear_sim_obstacles` |

Both halves are required. `gym_bridge` fixes the scan simulator's map at
`gym.make()` time and never re-reads it, so painting `/map` alone (which is what
`obstacle_publisher`'s `lidar` mode does) shows an obstacle in RViz that the
ego's LiDAR cannot see.

## Usage

Click **Publish Point** in RViz and pick a spot on the track. Clicking the same
spot again removes that obstacle. To clear all of them:

```bash
ros2 topic pub --once /clear_sim_obstacles std_msgs/msg/Empty {}
```

Obstacle size is the `obstacle_size` parameter on the `bridge` node — a cell
radius, default 2, so 4x4 cells at 0.05 m/cell = 0.2 m square. It is read once at
startup, so pass it at launch rather than with `ros2 param set`.

## Verified

With the stack running on hangar_1905_v0 and the ego on the raceline, a point
placed 1.61 m ahead shortened 37 beams to 1.25-1.30 m. That is the expected
geometry: the laser sits 0.27 m forward of `base_link`
(`scan_distance_to_base_link` in `config/SIM/sim.yaml`), so 1.61 - 0.27 = 1.34 m
to the obstacle centre and ~1.24 m to its near face.

**The simulation must be stepping for this to show up.** With `num_agent: 2`,
`gym_bridge` only calls `env.step()` once BOTH `/drive` and `/opp_drive` have
been published at least once. Until then the scan is frozen at its initial value
and the map updates silently have no visible effect — which looks exactly like
the feature being broken. Running the controller (head_to_head) and
`opponent_driver` satisfies both.

## Relation to the preserved patch

An earlier version of this work is on the `backup` remote as
`sim/preserved-work`; it additionally carried an InteractiveMarker "Clear
Obstacles" button, which was recorded as making `gym_bridge` fail at startup for
reasons never established. The button is deliberately absent here.
