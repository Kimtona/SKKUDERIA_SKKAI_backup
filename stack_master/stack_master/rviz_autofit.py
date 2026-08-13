"""Centre an RViz view on the map it is about to show.

RViz2 has no "zoom to fit": a view controller takes an explicit camera pose and
holds it, and there is no topic, service or parameter that moves the camera once
rviz is running. The only place a view can be chosen programmatically is the
config file rviz is started with. So this module reads the map, works out where
its centre is and how far back the camera has to sit to see all of it, and
writes a copy of an existing rviz config with those two things changed.

It changes as little as possible. The view controller, its pitch and yaw, and
every Display, topic, marker and panel setting are carried through untouched --
in particular the camera stays an Orbit, so the view can still be tilted, which
is how the global waypoint markers are read (global_planner_utils sets their
position.z and scale.z from vx_mps, so the velocity profile is only visible off
the vertical).

Used from a launch file:

    from stack_master.rviz_autofit import fitted_rviz_config
    cfg = fitted_rviz_config(template_path, map_yaml_path)

`fitted_rviz_config` never raises: if anything about the map cannot be read it
logs to stderr and hands back the template path unchanged, so a bad map can
leave the view where it was but can never stop rviz from starting.
"""

from __future__ import annotations

import math
import os
import sys
import tempfile

import yaml

# RViz renders with a 45 degree vertical field of view, so the camera sees
# 2 * tan(45/2) metres of ground per unit of Orbit Distance. Confirmed on this
# setup: an Orbit at Distance 40 showed 33.108 m of ground vertically, i.e.
# 0.82771 per unit, against the 0.828427 this constant gives.
GROUND_PER_DISTANCE = 2.0 * math.tan(math.radians(45.0) / 2.0)

# Chrome around the render panel, measured on the noVNC Xvfb (1920x1080): the
# render area sits at x 440..1889, y 127..1033, i.e. 1450x907. Only the aspect
# ratio is used below, but both are kept so the numbers stay checkable.
CHROME_W = 470
CHROME_H = 173

DEFAULT_SCREEN = (1920, 1080)

# Leave some air around the track rather than letting it touch the edges.
DEFAULT_MARGIN = 0.08


def _free_cell_bounds(map_yaml_path):
    """Return (x_min, y_min, x_max, y_max) in metres of the map's free space.

    Falls back to the full map extent when the image has no free cells, which is
    what an unthresholded or inverted map looks like.
    """
    from PIL import Image
    import numpy as np

    with open(map_yaml_path) as handle:
        meta = yaml.safe_load(handle)

    resolution = float(meta['resolution'])
    origin_x, origin_y = float(meta['origin'][0]), float(meta['origin'][1])
    image_path = meta['image']
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

    pixels = np.array(Image.open(image_path).convert('L'))
    rows, cols = pixels.shape

    full = (origin_x, origin_y,
            origin_x + cols * resolution, origin_y + rows * resolution)

    # nav2 map convention: occupancy p = (255 - pixel)/255 when negate is 0, and
    # a cell is free when p < free_thresh. Derive the cutoff from the map's own
    # thresholds rather than hardcoding a grey level.
    free_thresh = float(meta.get('free_thresh', 0.196))
    negate = int(meta.get('negate', 0))
    value = 255 - pixels if negate else pixels
    free = value > (255.0 * (1.0 - free_thresh))

    if not free.any():
        return full

    ys, xs = np.nonzero(free)
    # Row 0 of the image is the TOP of the map, i.e. the largest y.
    return (origin_x + xs.min() * resolution,
            origin_y + (rows - 1 - ys.max()) * resolution,
            origin_x + (xs.max() + 1) * resolution,
            origin_y + (rows - ys.min()) * resolution)


def _distance_for(across_m, down_m, aspect, margin):
    """Orbit Distance needed to show `across_m` x `down_m` of ground."""
    # Vertical FOV is fixed, so the horizontal one follows the aspect ratio.
    distance = max(down_m / GROUND_PER_DISTANCE,
                   across_m / (GROUND_PER_DISTANCE * aspect))
    return distance / max(1.0 - margin, 0.05)


def compute_view(map_yaml_path, screen=DEFAULT_SCREEN, margin=DEFAULT_MARGIN):
    """Return (focal_x, focal_y, distance, quarter_turn) framing the map.

    At the yaw the template carries, map x draws along screen x and map y along
    screen y -- measured against a map of known size, despite that yaw reading
    as 271 degrees. Adding pi/2 to it turns the view a quarter turn in plane.

    Screens are wider than they are tall, so a map that is taller than it is
    wide fits larger turned. `quarter_turn` says whether that is the case here;
    the caller adds pi/2 to the yaw when it is. hangar_1905_v0 is 7.10 x 23.80 m
    and gains 1.6x from the turn; the two GLC maps are already landscape and
    keep the saved orientation.
    """
    x_min, y_min, x_max, y_max = _free_cell_bounds(map_yaml_path)
    width_m = max(x_max - x_min, 1e-3)
    height_m = max(y_max - y_min, 1e-3)

    viewport_w = max(screen[0] - CHROME_W, 100)
    viewport_h = max(screen[1] - CHROME_H, 100)
    aspect = viewport_w / viewport_h

    upright = _distance_for(width_m, height_m, aspect, margin)
    turned = _distance_for(height_m, width_m, aspect, margin)

    quarter_turn = turned < upright
    distance = turned if quarter_turn else upright

    return ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, distance,
            quarter_turn)


def fitted_rviz_config(template_path, map_yaml_path, screen=DEFAULT_SCREEN,
                       margin=DEFAULT_MARGIN, out_path=None):
    """Write `template_path` recentred on the map; return the path to use.

    Returns `template_path` unchanged if anything cannot be read, so callers can
    use the result unconditionally.
    """
    try:
        with open(template_path) as handle:
            config = yaml.safe_load(handle)

        # Touch only where the camera is, how far back it sits, and whether the
        # map is turned to lie along the screen. Class, Pitch and everything
        # else stay exactly as the template has them -- Pitch in particular, so
        # the view is still tilted the same way and can still be tilted further.
        view = config['Visualization Manager']['Views']['Current']
        x, y, distance, quarter_turn = compute_view(
            map_yaml_path, screen=screen, margin=margin)
        view['Focal Point'] = {'X': float(x), 'Y': float(y), 'Z': 0.0}
        view['Distance'] = float(distance)
        if quarter_turn:
            view['Yaw'] = float(view.get('Yaw', 0.0)) + math.pi / 2.0

        if out_path is None:
            handle = tempfile.NamedTemporaryFile(
                mode='w', suffix='.rviz', prefix='sim_autofit_', delete=False)
            out_path = handle.name
        else:
            handle = open(out_path, 'w')
        with handle:
            yaml.safe_dump(config, handle, default_flow_style=False,
                           sort_keys=False)
    except Exception as exc:  # noqa: BLE001
        print(f'[rviz_autofit] falling back to {template_path}: {exc}',
              file=sys.stderr)
        return template_path

    print(f'[rviz_autofit] {os.path.basename(map_yaml_path)}: '
          f'Focal Point ({x:.3f}, {y:.3f}) Distance {distance:.2f}'
          f'{" quarter-turned" if quarter_turn else ""} -> {out_path}',
          file=sys.stderr)
    return out_path


def main():
    """CLI, for checking a map's fit without starting rviz."""
    import argparse

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('map_yaml')
    parser.add_argument('--template')
    parser.add_argument('--out')
    parser.add_argument('--screen-width', type=int, default=DEFAULT_SCREEN[0])
    parser.add_argument('--screen-height', type=int, default=DEFAULT_SCREEN[1])
    parser.add_argument('--margin', type=float, default=DEFAULT_MARGIN)
    args = parser.parse_args()

    screen = (args.screen_width, args.screen_height)
    x, y, distance, quarter_turn = compute_view(args.map_yaml, screen=screen,
                                                margin=args.margin)
    x_min, y_min, x_max, y_max = _free_cell_bounds(args.map_yaml)
    print(f'free space: x {x_min:.2f}..{x_max:.2f} ({x_max - x_min:.2f} m)  '
          f'y {y_min:.2f}..{y_max:.2f} ({y_max - y_min:.2f} m)')
    print(f'view: Focal Point ({x:.3f}, {y:.3f})  Distance {distance:.2f}  '
          f'quarter turn: {quarter_turn}')
    if args.template:
        print(fitted_rviz_config(args.template, args.map_yaml, screen=screen,
                                 margin=args.margin, out_path=args.out))


if __name__ == '__main__':
    main()
