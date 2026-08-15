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

These are `git am` patches, one per commit, stacked on a `skku/reactive-map` branch
in each submodule. **Apply them in the order below** — `apply.sh` already does, and
it is the order the commits were made in. Within `f1tenth_gym_ros` all four touch
`gym_bridge.py`, and the last one edits a block the first one introduces, so the
order is a requirement rather than a convention.

| # | Patch | Submodule | Adds |
|---|---|---|---|
| 1 | `f1tenth_gym__reactive-map.patch` | `f1tenth_gym` | `ScanSimulator2D.update_map_from_occupancy_grid()` |
| 2 | `f1tenth_gym__agent-collisions.patch` | `f1tenth_gym` | `RaceCar.agent_collisions` — optionally let hitting another car stop you |
| 3 | `f1tenth_gym_ros__reactive-map.patch` | `f1tenth_gym_ros` | `/map` subscribe + re-publish, `/clicked_point` obstacle toggle, `/clear_sim_obstacles`, `obstacle_size` |
| 4 | `f1tenth_gym_ros__agent-collisions.patch` | `f1tenth_gym_ros` | `agent_collisions` parameter on the bridge |
| 5 | `f1tenth_gym_ros__agent-collisions-launcharg.patch` | `f1tenth_gym_ros` | same, as a launch argument |
| 6 | `f1tenth_gym_ros__obstacle-size-param.patch` | `f1tenth_gym_ros` | `has_parameter` guard so `obstacle_size` can come from a params file |
| 7 | `f1tenth_gym_ros__rviz-autofit.patch` | `f1tenth_gym_ros` | launch-time RViz config fitted to the map being launched |
| 8 | `f1tenth_gym__scan-model.patch` | `f1tenth_gym` | `num_beams` / `fov` kwargs reach `RaceCar` instead of being ignored |
| 9 | `f1tenth_gym_ros__scan-model.patch` | `f1tenth_gym_ros` | pass `scan_beams` / `scan_fov` to the env; `angle_increment = fov / (n - 1)` |
| 10 | `f1tenth_gym_ros__scan-rate.patch` | `f1tenth_gym_ros` | `scan_rate_hz` — publish the scans on their own timer |

Patch 6 exists because the bridge is constructed with
`automatically_declare_parameters_from_overrides=True`: a parameter set in a params
file, on the command line or in a launch file is already declared before
`__init__` runs, so an unguarded `declare_parameter()` on it raises
`ParameterAlreadyDeclaredException` and the bridge dies at startup. Patch 3 declares
`obstacle_size` unguarded, and `config/SIM/sim.yaml` sets it (see below for why it
has to), so 3 without 6 will not launch. Patch 4 already had the guard for
`agent_collisions`; 6 gives `obstacle_size` the same treatment and changes nothing
when the parameter is left alone.

Both halves are required. `gym_bridge` fixes the scan simulator's map at
`gym.make()` time and never re-reads it, so painting `/map` alone (which is what
`obstacle_publisher`'s `lidar` mode does) shows an obstacle in RViz that the
ego's LiDAR cannot see.

Patches 8 and 9 make `scan_beams` and `scan_fov` mean something. `RaceCar` has
always taken `num_beams` and `fov`, but nothing passed them: `f110_env` did not
read them from kwargs and `Simulator` did not forward them, and the bridge used
its own two parameters only to stamp the `LaserScan` header. Setting them
therefore relabelled a scan that was still traced at the 1080-beam, 4.7 rad
default, and every consumer that turns an index into an angle read a field of
view that was not there. `config/SIM/sim.yaml` asks for 1501 beams over
4.712 rad, which is the `/scan` the real car publishes -- see the comment there.
Patch 9 also derives `angle_increment` as `fov / (n - 1)` rather than `fov / n`,
which is the spacing `ScanSimulator2D` traces at and the one that satisfies
`angle_min + (n-1) * angle_increment == angle_max`, as a real driver's scan does.

Patch 10 finishes the sensor off. Scans went out on the same 100 Hz timer as the
odometry, so their rate was the sim step rate; the GL-5 turns at 40 Hz, and
anything acting once per scan was getting 2.5x its real reaction time.
`scan_rate_hz` gives the two `LaserScan` topics their own timer and touches
nothing else — the step, the odometry and the transforms stay at 100 Hz. It
defaults to 100.0, i.e. the old behaviour, and `config/SIM/sim.yaml` sets 40.0.

## Usage

Click **Publish Point** in RViz and pick a spot on the track. Clicking the same
spot again removes that obstacle. To clear all of them:

```bash
ros2 topic pub --once /clear_sim_obstacles std_msgs/msg/Empty {}
```

Obstacle size is the `obstacle_size` parameter on the `bridge` node — a cell
radius, so the painted block is `(2 * obstacle_size)^2` cells at 0.05 m/cell. It is
read once at startup, so set it in a params file or pass it at launch rather than
with `ros2 param set`.

**The bridge's own default of 2 (0.20 m) is too small to be usable**, which is why
`config/SIM/sim.yaml` sets `obstacle_size: 4` (0.40 m). A 0.20 m obstacle is seen
by the LiDAR and still invisible to the stack: `detect` segments the scan and drops
any cluster below `min_obs_size` = 10 laser points. With gym's 4.7 rad over 1080
beams = 0.00435 rad per beam, an obstacle of side `w` at range `r` spans
`w / (r * 0.00435)` points, so

| side | detectable out to | verdict |
|---|---|---|
| 0.20 m | 4.6 m | measured 8 points at 5.5 m; `/perception/obstacles` stayed empty |
| 0.40 m | 9.2 m | measured size 0.385 m, detected 67-98% of cycles |

0.40 m is the practical choice: it covers `detect`'s `max_viewing_distance` of
9.0 m while staying under `max_obs_size` = 0.5 m. Before this default was raised,
placing a usable obstacle took a 2x2 block of clicks.

Note that `detect` reporting nothing looks exactly like the map patch not working.
The way to tell them apart is that the beam count drops but
`/perception/detection/raw_obstacles` stays empty — see the verification below.

## Seeing the evasion path in RViz

`config/SIM/sim.rviz` had no planner display at all, so the purple evasion spline
could never appear however well the rest of the chain worked. It now carries a
**Planner** group, enabled:

| Display | Topic | What it looks like |
|---|---|---|
| Avoidance | `/planner/avoidance/markers` | purple cylinders (r 0.75 / b 0.75), height scaled by speed |
| Considered OBS | `/planner/avoidance/considered_OBS` | teal sphere on the obstacle the planner is reacting to |

`viz/head_to_head.rviz` ships both `Enabled: false`; they are on here because
whether an evasion exists is the whole point of these tests.

The chain has four links and each fails differently, so check them in order:

```
/clicked_point -> /map            occupied cell count rises by (2*obstacle_size)^2
/map           -> /scan           beams shorten (needs the sim to be STEPPING)
/scan          -> raw_obstacles   detect, gated by min_obs_size / max_obs_size
raw_obstacles  -> markers         tracking, then spline_planner
```

Two traps when checking this by hand:

- **The scan only updates inside `env.step()`**, which with `num_agent: 2` needs
  both `/drive` and `/opp_drive`. A frozen sim shows the new `/map` in RViz and an
  unchanged `/scan`, which looks like the patch not working. To compare scans
  meaningfully the ego must be still *and* the sim stepping — stop the `controller`
  node and publish zeros to both drive topics.
- **The scan carries noise of about ±0.06 m**, so a "shortened beams" threshold of
  0.05 m reports beams all over the scan. Use a threshold well above the noise, or
  restrict the comparison to beams near straight ahead.

**Detection is intermittent on this machine, and it is a TF problem, not a size
problem.** `detect` gives up on a whole cycle when it cannot transform the scan
into `map`:

```
Could not transform between 'map' and 'car_state/laser', latest TF is 183 ms older than scan
[DEBUG] [Opponent Detection] detected 0 raw obstacles.
```

Its fallback accepts a stale TF only within `min(0.15, 0.3 / speed)` seconds, so a
*stationary* car gets the 150 ms bound and TF running ~180 ms behind under load is
rejected. A steady obstacle therefore shows up in only 67-98% of cycles. That
flicker is what stops `tracking` from accumulating `min_nb_meas` (5) measurements,
which is in turn why its static/dynamic flag is unreliable in both directions —
a parked obstacle has been seen published as dynamic, and a car moving at 1.5 m/s
was never published as dynamic at all. The markers still render continuously
because RViz keeps the last `MarkerArray`.

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
