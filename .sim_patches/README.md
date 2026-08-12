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
| `f1tenth_gym__agent-collisions.patch` | `RaceCar.agent_collisions` — optionally let hitting another car stop you |
| `f1tenth_gym_ros__agent-collisions.patch` | `agent_collisions` parameter on the bridge |
| `f1tenth_gym_ros__agent-collisions-launcharg.patch` | same, as a launch argument |

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

## Agent collisions

Upstream `update_scan()` runs the iTTC collision check on the map-only scan and
only then ray casts the other agents in, so **a car drives straight through
another car with no effect on its motion**. In simulation that makes a failed
overtake invisible: laps still complete, nothing is logged, and only measuring
centre-to-centre distance reveals it. That is why the avoidance checks in
`measure_full.py` are geometric and must stay that way — with collisions off,
the simulator will never report a bad overtake.

`agent_collisions` on the `bridge` node flips the order so the check sees the
other agents. **Default false**, i.e. upstream behaviour, so no existing launch
changes. The disabled branch keeps the original call order verbatim rather than
assuming `ray_cast_agents` leaves its input untouched.

```bash
ros2 launch stack_master base_system_launch.xml \
    sim:=True agent_collisions:=true map_name:=hangar_1905_v0 racecar_version:=SIM
```

`base_system_launch.xml` forwards the argument to `gym_bridge_launch.py`, which
passes it to the node after `sim.yaml`, so the launch argument wins over anything
set in that file. It is read once at startup.

Enabling it means a car that hits the opponent stays stopped until an
`/initialpose` reset, which makes repeated runs more work — that is the reason
it is opt-in rather than on.

Verified by driving the ego down a straight stretch of hangar_1905_v0 (s = 11.1,
curvature 0.0005 rad/m) into a parked opponent at y = 10.196, with the controller
stopped and `/drive` published directly:

| | ego trajectory |
|---|---|
| `false` | 7.70 -> 8.79 -> **10.22 (straight through)** -> 11.73 -> ... -> 20.48, speed held at 1.5 |
| `true`  | 7.70 -> 8.73 -> **stops at 9.735**, speed 0.000 thereafter |

A control run with the opponent moved away covered the same 12.4 m without
stopping, confirming the stop is the opponent and not a wall.

**Watch out when testing this by hand:** `gym_bridge` keeps the last commanded
speed, so resetting with a stale non-zero `/drive` sends the car off the instant
it is placed. It then hits a wall, latches `in_collision`, and ignores every later
command — which looks exactly like this feature misbehaving. Command zero first,
then reset.

## Relation to the preserved patch

An earlier version of this work is on the `backup` remote as
`sim/preserved-work`; it additionally carried an InteractiveMarker "Clear
Obstacles" button, which was recorded as making `gym_bridge` fail at startup for
reasons never established. The button is deliberately absent here.
