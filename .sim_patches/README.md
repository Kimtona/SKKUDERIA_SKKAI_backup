# Simulator submodule patches

`f1tenth_gym` and `f1tenth_gym_ros` point at ForzaETH upstream, which we cannot
push to, so changes to them live here as patch files and are applied after
`git submodule update --init`.

They apply onto the ForzaETH upstream commits `f8f8c9d` (`f1tenth_gym`) and
`6ffd209` (`f1tenth_gym_ros`):

```bash
./.sim_patches/apply.sh
```

`apply.sh` is safe to re-run — it skips patches already present — and it leaves
each submodule on a local `skku/reactive-map` branch.

Since `ee4b684` the superproject records the *patched* tips rather than those
upstream SHAs, so that a clone does not silently come up with an unpatched
simulator. Those tips only exist locally: we cannot push to ForzaETH, so
`git submodule update --init` on a new machine cannot fetch them and will fail
on the recorded SHA. Check the upstream SHA out by hand there and run `apply.sh`:

```bash
git -C base_system/f110_simulator/f1tenth_gym     checkout f8f8c9d
git -C base_system/f110_simulator/f1tenth_gym_ros checkout 6ffd209
./.sim_patches/apply.sh
```

## What they add

Placing a static obstacle by clicking **Publish Point** in RViz.

These are `git am` patches, one per commit, stacked on a `skku/reactive-map` branch
in each submodule. **Apply them in the order below** — `apply.sh` already does, and
it is the order the commits were made in. Five of the seven `f1tenth_gym_ros`
patches touch `gym_bridge.py` (the other two touch `launch/gym_bridge_launch.py`),
and `obstacle-size-param` edits a block `reactive-map` introduces, so the order is
a requirement rather than a convention.

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
any cluster below `min_obs_size` = 10 laser points -- `min_points` in
`detection_core.cpp` for the C++ detector, the same gate in `detect.py` for the
legacy one. An obstacle of side `w` at range `r` spans roughly
`w / (r * angle_increment)` points, so the reach follows the scan model:

| side | reach at 0.00435 rad (1080 beams) | reach at 0.0031413 rad (1501 beams, current) |
|---|---|---|
| 0.20 m | 4.6 m | 6.4 m |
| 0.40 m | 9.2 m | 12.7 m |

The measurements behind this were taken on the 1080-beam scan, before patches 8
and 9: a 0.20 m obstacle gave 8 points at 5.5 m with `/perception/obstacles`
staying empty, and a 0.40 m one fitted at 0.385 m and was detected in 67-98% of
cycles. The finer scan only widens both margins, so they were not repeated.

0.40 m is still the practical choice: at either scan model it covers `detect`'s
`max_viewing_distance` of 9.0 m while staying under `max_obs_size` -- 0.5 m in the
shared config, 0.7 m under `config/SIM/sim_perception_overrides.yaml`. Before this
default was raised, placing a usable obstacle took a 2x2 block of clicks.

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

**Detection was intermittent on this machine, and it was a TF problem, not a size
problem.** `detect` gives up on a whole cycle when it cannot transform the scan
into `map`:

```
Could not transform between 'map' and 'car_state/laser', latest TF is 183 ms older than scan
[DEBUG] [Opponent Detection] detected 0 raw obstacles.
```

Its fallback accepts a stale TF only within `min(0.15, 0.3 / speed)` seconds, so a
*stationary* car gets the 150 ms bound and TF running ~180 ms behind under load is
rejected. A steady obstacle showed up in only 67-98% of cycles, and that flicker
was the explanation for `tracking` failing to accumulate `min_nb_meas` (5)
measurements — why its static/dynamic flag was unreliable in both directions, with
a parked obstacle published as dynamic and a car moving at 1.5 m/s never published
as dynamic at all. The markers still render continuously because RViz keeps the
last `MarkerArray`.

Those figures predate patches 8 and 9. On the 1501-beam scan a *parked* opponent
is published at its true position in 100% of messages (verified in `19b66a1` at
s = 8.1, d = +0.02), so intermittency is no longer the first thing to suspect. A
*driven* opponent is a different case and still intermittent: about 72% of
messages carry an obstacle at its true position, measured across the `ttl_static`
sweeps. A large part of that gap was `max_obs_size` clipping the fit rather than
TF — see `config/SIM/sim_perception_overrides.yaml`, which raises it to 0.7 for
sim — and what remains has not been traced to a cause.

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
centre-to-centre distance reveals it. That is why any avoidance check has to be
geometric — with collisions off, the simulator will never report a bad overtake.
(The `measure_full.py` this used to name is not in this repo and is not on the
host; treat the rule, not the file, as the thing to keep.)

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
