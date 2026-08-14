#!/usr/bin/env python3
"""Turn a hand-drawn track sketch into an F1TENTH occupancy map.

    python3 sketch_to_map.py sketch.png my_track --seed 400 420 \
        --box 29 842 148 445 --box-size 20 8 --canvas 22 8 --start 615 --heading 0.0

That exact line rebuilds skku_track_22x8 from maps/skku_track_22x8/sketch.png.
`--heading 0.0` is the opposite of the traced tangent there, which is what makes
the car run the main straight in +x.

The sketch is an ordinary drawing: two closed outlines (outer wall and inner
island) with the drivable ribbon between them, plus a rectangle annotating the
real-world size of the room. Anything darker than INK_THRESH counts as a wall.

Resampling the sketch directly does not work -- its walls are one or two pixels
wide, which is under one cell at 0.05 m, and its concave pockets turn into
skeleton branches that `global_planner`'s `extract_centerline` cannot resolve.
So the drawing is reduced to a centre line plus a width profile and the ribbon
is re-rendered from those:

  1. binarise -> flood-fill the drivable ribbon from a seed inside it
  2. skeletonise, prune every spur until only the cycle is left
  3. walk the cycle into an ordered polyline (diagonal edges dropped where a
     4-connected detour exists, so every node has degree 2)
  4. scale to metres from the annotation box, resample, low-pass the hand jitter
  5. read the half-width off the distance transform at each centre-line point
  6. rasterise: a cell is free iff it is within half-width of the centre line

Pockets vanish in step 6 and the hairline between two passes becomes a wall as
thick as 2*TRIM. Widths are clamped so the track never drops below MIN_WIDTH.

Because the width only enters at step 6, `--width` can replace the profile from
step 5 with one full width per sector and leave the centre line -- the shape of
the track -- exactly where it was. That is how skku_track_22x8_wide / _mid /
_narrow are built; `build_track_variant.sh` runs this and everything downstream
of it. Two things bound how wide a sector can go, and both are checked here: the
ribbon has to stay inside the canvas, and the wall between two passes has to
survive. On this sketch that leaves roughly +0.2 m over the _mid set before the
track runs out of the 8 m room, and the wall down to 0.30 m.

Writes <name>.png and <name>.yaml into stack_master/maps/<name>/. The YAML
carries `initial_pose`, which global_planner requires and which no map produced
by mapping_node's current writer path is guaranteed to have. Then:

    ln -sfn ~/ws/src/race_stack/stack_master ~/ws/src/stack_master   # see below
    MPLBACKEND=Agg python3 -c "import matplotlib.pyplot as plt; plt.show = \
        lambda *a, **k: None; from global_planner.global_planner_node import \
        main; main()" --ros-args -p map_name:=my_track -p safety_width:=0.7
    colcon build --packages-select stack_master --symlink-install

The symlink is needed because global_planner's `get_data_path()` resolves to
<ws>/src/stack_master while the repo is mounted at <ws>/src/race_stack, and the
plt patch is needed because `load_and_plot_map()` plots unconditionally and
blocks on QtAgg. Finally copy speed_scaling.yaml / ot_sectors.yaml from another
map, renumber the sectors to the new waypoint count, and re-derive the sim spawn
poses in config/SIM/sim.yaml from the new global_waypoints.json.
"""
import argparse
import json
import os

import cv2
import numpy as np
import yaml
from scipy import ndimage
from scipy.spatial import cKDTree
from skimage.morphology import skeletonize

# --- tuning ---------------------------------------------------------------
INK_THRESH = 190     # below this grey level a sketch pixel is a wall
RESOLUTION = 0.05    # m per cell, matches every other map in stack_master
SCALE = 1.10         # blow the traced sketch up by this before rendering
TRIM = 0.075         # m shaved off each side; separating walls grow by 2*TRIM
MIN_WIDTH = 1.00     # m, hard floor on the rendered track width
SMOOTH_C = 60        # Fourier terms kept for the centre line
SMOOTH_W = 40        # Fourier terms kept for the width profile
STEP = 0.02          # m, centre-line resampling pitch


def drivable_mask(sketch_path, seed_xy):
    """Binarise the sketch and flood-fill the ribbon containing seed_xy."""
    img = cv2.imread(sketch_path, cv2.IMREAD_GRAYSCALE)
    if img is None:
        raise SystemExit(f'cannot read {sketch_path}')
    # dilate the ink by one pixel first: hand-drawn strokes have gaps, and a
    # single leaking pixel merges the ribbon with the outside and the fill
    # swallows the whole image.
    ink = ndimage.binary_dilation(img < INK_THRESH, np.ones((3, 3)))
    labels, _ = ndimage.label(~ink)
    seed = labels[seed_xy[1], seed_xy[0]]
    if seed == 0:
        raise SystemExit(f'seed {seed_xy} landed on ink, not inside the track')
    return labels == seed


def centreline_cycle(mask):
    """Skeletonise, prune to the cycle, and return it as ordered (row, col)."""
    sk = skeletonize(mask)
    nb = np.ones((3, 3)); nb[1, 1] = 0
    while True:                                   # peel spurs until none remain
        ends = sk & (ndimage.convolve(sk.astype(np.uint8), nb, mode='constant') <= 1)
        if not ends.any():
            break
        sk &= ~ends

    pts = set(map(tuple, np.argwhere(sk)))
    nb4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
    adj = {p: set() for p in pts}
    for p in pts:
        for d in nb4:
            q = (p[0] + d[0], p[1] + d[1])
            if q in pts:
                adj[p].add(q); adj[q].add(p)
    for p in pts:                                 # keep a diagonal only when it
        for d in [(-1, -1), (-1, 1), (1, -1), (1, 1)]:   # is the only way across
            q = (p[0] + d[0], p[1] + d[1])
            if q in pts and (p[0], q[1]) not in pts and (q[0], p[1]) not in pts:
                adj[p].add(q); adj[q].add(p)

    start = next(p for p in pts if len(adj[p]) == 2)
    path, seen = [start], {start}
    prev, cur = start, next(iter(adj[start]))
    path.append(cur); seen.add(cur)
    while True:
        nxt = [q for q in adj[cur] if q != prev and q not in seen]
        if not nxt:
            break
        prev, cur = cur, min(nxt, key=lambda q: abs(q[0] - cur[0]) + abs(q[1] - cur[1]))
        path.append(cur); seen.add(cur)
    if len(path) < 0.9 * len(pts):
        raise SystemExit(f'cycle walk covered {len(path)}/{len(pts)} skeleton px -- '
                         'the drawing probably has a branch or a gap')
    return np.array(path, float)


def lowpass(a, keep):
    f = np.fft.rfft(a, axis=0)
    f[keep:] = 0
    return np.fft.irfft(f, n=len(a), axis=0)


def to_metres(rc, mask, box, box_size):
    """Map the traced loop to metres and sample its half-width."""
    x0, x1, y0, y1 = box
    sx, sy = box_size[0] / (x1 - x0), box_size[1] / (y1 - y0)
    xy = np.stack([(rc[:, 1] - x0) * sx, (y1 - rc[:, 0]) * sy], 1)

    xy = np.vstack([xy, xy[:1]])
    d = np.r_[0, np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]
    s = np.linspace(0, d[-1], int(round(d[-1] / STEP)), endpoint=False)
    c = lowpass(np.stack([np.interp(s, d, xy[:, 0]), np.interp(s, d, xy[:, 1])], 1), SMOOTH_C)

    dt = ndimage.distance_transform_edt(mask)
    col = np.clip(c[:, 0] / sx + x0, 0, mask.shape[1] - 1).astype(int)
    row = np.clip(y1 - c[:, 1] / sy, 0, mask.shape[0] - 1).astype(int)
    return c, lowpass(dt[row, col] * 0.5 * (sx + sy), SMOOTH_W)


def drive_frac(n, start, reverse):
    """Lap fraction of each centre-line index, measured in driving order."""
    i = np.arange(n)
    return ((start - i) % n) / n if reverse else ((i - start) % n) / n


def sector_halfwidth(n, lap, start, reverse, bounds, widths, taper):
    """Piecewise-constant per-sector width, blended over `taper` metres."""
    fr = drive_frac(n, start, reverse)
    hw = np.empty(n)
    for k, a in enumerate(bounds):
        b = bounds[(k + 1) % len(bounds)]
        sel = (fr >= a) & (fr < b) if a < b else (fr >= a) | (fr < b)
        hw[sel] = widths[k] / 2
    # A step in the width profile puts a step in the wall. Blend it out with a
    # Hann window as long as `taper`, wrapped so the start/finish joint is
    # smooth too. Sectors here are 9-15 m, so a 2 m blend leaves the interiors
    # at their nominal width.
    m = max(3, int(round(taper / (lap / n))) | 1)
    win = np.hanning(m + 2)[1:-1]
    return np.convolve(np.r_[hw[-m:], hw, hw[:m]], win / win.sum(), 'same')[m:-m]


def wall_report(free, resolution=RESOLUTION):
    """Per-wall-component area and thinnest neck, outer wall excluded."""
    walls, n = ndimage.label(~free)
    border = set(np.r_[walls[0], walls[-1], walls[:, 0], walls[:, -1]]) - {0}
    out = []
    for lab in range(1, n + 1):
        if lab in border:                         # the outside, unbounded
            continue
        blob = walls == lab
        dt = ndimage.distance_transform_edt(np.pad(blob, 1))[1:-1, 1:-1]
        # thinnest cross-section = smallest 2*dt along the medial axis; the
        # skeleton's own tips read low, so ignore the outer 10% of it.
        sk = skeletonize(blob)
        neck = np.sort(2 * dt[sk] * resolution)
        out.append((blob.sum() * resolution ** 2,
                    float(neck[max(0, int(0.1 * len(neck)))]) if len(neck) else 0.0))
    return out


def render(c, hw, canvas):
    """Rasterise the ribbon, centred in a canvas of (width, height) metres."""
    lo = np.array([(c[:, 0] - hw).min(), (c[:, 1] - hw).min()])
    hi = np.array([(c[:, 0] + hw).max(), (c[:, 1] + hw).max()])
    if np.any(hi - lo > np.array(canvas)):
        over = np.maximum(hi - lo - np.array(canvas), 0)
        raise SystemExit(f'track is {(hi - lo).round(2)} m, over a {canvas} m canvas '
                         f'by {over.round(2)} m -- narrow a sector or grow --canvas')
    c = c + (np.array(canvas) - (hi + lo)) / 2

    closed = np.vstack([c, c[:1]])
    t = np.r_[0, np.cumsum(np.hypot(*np.diff(closed, axis=0).T))]
    tn = np.arange(0, t[-1], RESOLUTION / 4)      # dense enough that the
    dense = np.stack([np.interp(tn, t, closed[:, 0]),   # nearest-point lookup
                      np.interp(tn, t, closed[:, 1])], 1)  # is exact per cell
    dense_hw = np.interp(tn, t, np.r_[hw, hw[0]])

    w, h = (int(round(v / RESOLUTION)) for v in canvas)
    gx, gy = np.meshgrid((np.arange(w) + 0.5) * RESOLUTION, (np.arange(h) + 0.5) * RESOLUTION)
    dist, idx = cKDTree(dense).query(np.stack([gx.ravel(), gy.ravel()], 1))
    free = (dist <= dense_hw[idx]).reshape(h, w)
    return free, c, hw, t[-1]


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('sketch')
    p.add_argument('name', help='map name; output goes to stack_master/maps/<name>/')
    p.add_argument('--seed', type=int, nargs=2, required=True, metavar=('X', 'Y'),
                   help='pixel inside the drivable ribbon of the sketch')
    p.add_argument('--box', type=int, nargs=4, required=True, metavar=('X0', 'X1', 'Y0', 'Y1'),
                   help='pixel bounds of the annotation rectangle')
    p.add_argument('--box-size', type=float, nargs=2, default=(20.0, 8.0), metavar=('W', 'H'),
                   help='real size of that rectangle in metres')
    p.add_argument('--canvas', type=float, nargs=2, default=(22.0, 8.0), metavar=('W', 'H'),
                   help='output map size in metres')
    p.add_argument('--start', type=int, default=0,
                   help='centre-line index put at map-frame (0, 0); pick one on a straight')
    p.add_argument('--heading', type=float, default=None,
                   help='initial_pose yaw in rad. Defaults to the traced tangent; pass the '
                        'opposite to make global_planner run the lap the other way round.')
    p.add_argument('--width', type=float, nargs='+', metavar='W',
                   help='one full track width in metres per sector, in driving order from '
                        '--start. Replaces the width profile read off the sketch; the centre '
                        'line, and so the shape of the track, is untouched.')
    p.add_argument('--sector-bounds', type=float, nargs='+', metavar='F',
                   default=[0.0639, 0.3768, 0.5672, 0.8029],
                   help='lap fraction where each --width sector begins, driving order from '
                        '--start. The default four are skku_track_22x8: main straight, right '
                        'hairpin, middle corridor, left S-bend.')
    p.add_argument('--taper', type=float, default=2.0,
                   help='metres over which one sector width blends into the next')
    p.add_argument('--min-width', type=float, default=MIN_WIDTH,
                   help='hard floor on the rendered track width in metres')
    p.add_argument('--expect-walls', type=int, default=None,
                   help='fail unless the map ends up with this many inner islands; a widened '
                        'sector that swallows the wall between two passes drops the count')
    p.add_argument('--min-wall', type=float, default=0.10,
                   help='fail if any inner island is thinner than this, in metres')
    args = p.parse_args()
    if args.width and len(args.width) != len(args.sector_bounds):
        raise SystemExit(f'{len(args.width)} widths for {len(args.sector_bounds)} sectors')

    mask = drivable_mask(args.sketch, args.seed)
    c, hw = to_metres(centreline_cycle(mask), mask, args.box, args.box_size)

    c = c * SCALE
    hw = np.maximum(hw * SCALE - TRIM, args.min_width / 2)
    tangent = c[(args.start + 5) % len(c)] - c[args.start]
    traced = float(np.arctan2(tangent[1], tangent[0]))
    # Driving order is the traced order unless --heading opposes it, and every
    # sector fraction below is measured in driving order.
    reverse = args.heading is not None and np.cos(args.heading - traced) < 0
    if args.width:
        lap = float(np.hypot(*np.diff(np.vstack([c, c[:1]]), axis=0).T).sum())
        hw = np.maximum(sector_halfwidth(len(c), lap, args.start, reverse,
                                         args.sector_bounds, args.width, args.taper),
                        args.min_width / 2)
    free, c, hw, lap = render(c, hw, args.canvas)

    holes, n_holes = ndimage.label(~free)
    n_free = ndimage.label(free)[1]
    if n_free != 1:
        raise SystemExit(f'{n_free} disconnected free regions -- the ribbon self-intersects')
    print(f'lap length     {lap:.2f} m')
    print(f'track width    min {2 * hw.min():.2f}  median {2 * np.median(hw):.2f}  '
          f'max {2 * hw.max():.2f} m')
    print(f'free regions   {n_free} (must be 1)')
    print(f'wall regions   {n_holes} (outside + inner islands)')

    islands = wall_report(free)
    for i, (area, neck) in enumerate(islands):
        print(f'  island {i}     {area:.2f} m2, thinnest {neck:.2f} m')
    if args.expect_walls is not None and len(islands) != args.expect_walls:
        raise SystemExit(f'{len(islands)} inner islands, expected {args.expect_walls} -- a '
                         'sector is wide enough to swallow the wall between two passes')
    thin = [n for _, n in islands if n < args.min_wall]
    if thin:
        raise SystemExit(f'wall down to {min(thin):.2f} m, under --min-wall {args.min_wall}')

    out = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps', args.name)
    os.makedirs(out, exist_ok=True)
    cv2.imwrite(os.path.join(out, args.name + '.png'),
                np.flipud(np.where(free, 255, 0).astype(np.uint8)))

    sx, sy = c[args.start]
    # Where each sector begins, in map frame. The raceline does not exist yet and
    # its waypoint 0 lands wherever global_planner puts it, so sector indices are
    # recovered later by projecting these points onto it -- see map_sectors.py.
    fr = drive_frac(len(c), args.start, reverse)
    starts = [c[int(np.argmin(np.abs(fr - f)))] - [sx, sy] for f in args.sector_bounds]
    with open(os.path.join(out, 'sector_bounds.json'), 'w') as f:
        json.dump({'bounds_frac': list(args.sector_bounds),
                   'bounds_xy': [[round(float(x), 4), round(float(y), 4)] for x, y in starts],
                   'widths': list(args.width) if args.width else None,
                   'lap_m': round(float(lap), 3)}, f, indent=1)

    if args.heading is None:
        args.heading = round(traced, 4)
    with open(os.path.join(out, args.name + '.yaml'), 'w') as f:
        yaml.dump({'image': args.name + '.png',
                   'resolution': RESOLUTION,
                   # plain floats, not numpy scalars: yaml.dump serialises those
                   # as !!python/object/apply and map_server cannot read it back
                   'origin': [round(float(-sx), 4), round(float(-sy), 4), 0],
                   'negate': 0,
                   'occupied_thresh': 0.65,
                   'free_thresh': 0.196,
                   'initial_pose': [0.0, 0.0, args.heading]},
                  f, default_flow_style=False)
    print(f'wrote          {os.path.normpath(out)}/{args.name}.{{png,yaml}}')


if __name__ == '__main__':
    main()
