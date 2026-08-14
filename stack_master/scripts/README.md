# Usage of the different Scripts

## Run a map in the simulator
```bash
./sim_map.sh <map_name>                    # time trials
./sim_map.sh <map_name> head_to_head       # overtake stack, needs num_agent 2
```
Starts the Zenoh router, puts that map's spawn poses in `config/SIM/sim.yaml`,
launches the base system and the chosen stack, and kicks `/opp_drive` when
sim.yaml asks for two cars. Ctrl-C stops it. It refuses to start on top of a
running stack and says what to kill.

The maps in `stack_master/maps/`:

| map | source | notes |
| --- | --- | --- |
| `hangar_1905_v0` | recorded | 7.10 x 23.80 m |
| `GLC_smile_small`, `glc_ot_ez` | recorded | |
| `skku_track_22x8` | `sketch_to_map.py` | 48.0 m lap, 1.01-2.16 m wide |
| `skku_track_22x8_wide` | `build_track_variant.sh` | 2.20 / 1.65 / 1.60 / 1.75 m |
| `skku_track_22x8_mid` | `build_track_variant.sh` | 1.95 / 1.40 / 1.35 / 1.50 m |
| `skku_track_22x8_narrow` | `build_track_variant.sh` | 1.70 / 1.15 / 1.10 / 1.25 m |

The last three are one shape at three widths, one width per sector in driving
order from the start line: main straight, right hairpin, middle corridor, left
S-bend. Side by side in `maps/skku_track_22x8/width_variants.png`.

## Build a track at a different width
```bash
./build_track_variant.sh <new_map_name> <w0> <w1> <w2> <w3>
```
Renders the map from the committed sketch at those four sector widths, builds
its raceline, writes its sector YAMLs and rebuilds `stack_master`. The centre
line is untouched, so every variant is the same track shape. It stops with a
message if a width does not fit the room or eats the wall between two passes;
`skku_track_22x8_wide` is roughly the ceiling. See the docstrings in
`sketch_to_map.py` and `map_sectors.py`.

## Convert Old Wpnts
If you want to use old racelines from the ROS1 Racestack, the ROS2 messages need to be adapted, use this script to apply this. This script transforms `global_waypoints.json` files within map directories located at `~/ws/data/maps`. It modifies `header` and `lifetime` objects in the JSON structure.

## Prerequisites
- `jq` must be installed.

## Usage
Run the script with the name of the map directory:
```bash
./convert_old_gbwpnts.sh <map_name>
```

## Example
```bash
./convert_old_gbwpnts.sh GLC_smile_small
```

## Figure out which maps are there
This will list the maps in the `~/ws/data/maps` directory:
```bash
./convert_old_gbwpnts.sh
```