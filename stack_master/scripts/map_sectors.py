#!/usr/bin/env python3
"""Write a map's sector YAMLs and print its sim spawn poses.

    python3 map_sectors.py skku_track_22x8_narrow

Run it after `global_planner` has written `global_waypoints.json`. It needs the
`sector_bounds.json` that `sketch_to_map.py` leaves next to the map.

Sector boundaries have to be waypoint indices, but the raceline is rebuilt for
every map and a wider track moves every index -- the same corner is waypoint 160
on one map and 171 on the next. So the boundaries are carried between maps as
map-frame points and projected onto whichever raceline is on disk.

Sector0 starts at waypoint 0 rather than at its own stored point, because
`sector_tuner` and `ot_interpolator` both expect the sectors to tile 0..N-1 in
order. The two agree to within a metre as long as the stored point sits on the
start straight; the script says how far apart they came out.
"""
import argparse
import json
import os

import numpy as np

SIM_OPP_S = 8.0     # m down the raceline, where sim.yaml parks the opponent


def load(map_dir):
    with open(os.path.join(map_dir, 'global_waypoints.json')) as f:
        wpnts = json.load(f)['global_traj_wpnts_iqp']['wpnts']
    with open(os.path.join(map_dir, 'sector_bounds.json')) as f:
        bounds = json.load(f)
    return wpnts, bounds


def sector_indices(wpnts, bounds_xy):
    """Nearest waypoint to each stored boundary point, Sector0 pinned to 0."""
    xy = np.array([[w['x_m'], w['y_m']] for w in wpnts])
    idx = [int(np.argmin(((xy - np.array(b)) ** 2).sum(1))) for b in bounds_xy]
    drift = float(np.hypot(*(xy[idx[0]] - xy[0])))
    starts = [0] + idx[1:]
    if sorted(starts) != starts or len(set(starts)) != len(starts):
        raise SystemExit(f'boundaries came out as {starts}, not in lap order -- the stored '
                         'points do not project onto this raceline in the driving direction')
    return starts, drift


def sector_block(wpnts, starts, per_sector):
    """One dict entry per sector, plus the s range each covers as a comment."""
    ends = [s - 1 for s in starts[1:]] + [len(wpnts) - 1]
    return [dict(name=f'Sector{i}', start=a, end=b,
                 s=(wpnts[a]['s_m'], wpnts[b]['s_m']), **per_sector)
            for i, (a, b) in enumerate(zip(starts, ends))]


def dump(path, node, header, extra, sectors, keys):
    lines = [header, f'{node}:', '  ros__parameters:']
    lines += [f'    {k}: {v}' for k, v in extra.items()]
    lines.append(f'    n_sectors: {len(sectors)}')
    for s in sectors:
        lines.append(f"    {s['name']}:  # s {s['s'][0]:.1f}-{s['s'][1]:.1f} m")
        for k in keys:
            v = s[k]
            lines.append(f'      {k}: {str(v).lower() if isinstance(v, bool) else v}')
    with open(path, 'w') as f:
        f.write('\n'.join(lines) + '\n')


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('name', help='map name under stack_master/maps/')
    p.add_argument('--scaling', type=float, default=0.5, help='speed scaling per sector')
    p.add_argument('--spawn-only', action='store_true',
                   help='leave the sector YAMLs alone. They are rewritten on every other '
                        'run, which throws away any per-sector scaling tuned by hand.')
    p.add_argument('--set-spawn', action='store_true',
                   help='also write the spawn poses into config/SIM/sim.yaml, so the sim '
                        'launches on this map. Off by default: rebuilding a map should not '
                        'silently repoint the simulator at it.')
    p.add_argument('--ot-flag', action='store_true',
                   help='enable overtaking in every sector. Off by default, as every map '
                        'ships, so the real car cannot overtake by accident -- simulation '
                        'passes force_all_ot_true instead.')
    args = p.parse_args()

    root = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'maps', args.name)
    wpnts, bounds = load(root)
    starts, drift = sector_indices(wpnts, bounds['bounds_xy'])
    widths = bounds['widths']

    header = (f'# Written by map_sectors.py from {args.name}\'s IQP raceline '
              f'({len(wpnts)} waypoints, {wpnts[-1]["s_m"]:.1f} m).'
              + (f'\n# Sector widths {widths} m.' if widths else ''))
    if not args.spawn_only:
        dump(os.path.join(root, 'speed_scaling.yaml'), 'sector_tuner', header,
             {'global_limit': 0.5},
             sector_block(wpnts, starts,
                          dict(scaling=args.scaling, only_FTG=False, no_FTG=False)),
             ['start', 'end', 'scaling', 'only_FTG', 'no_FTG'])
        dump(os.path.join(root, 'ot_sectors.yaml'), 'ot_interpolator', header,
             {'yeet_factor': 1.25, 'spline_len': 30, 'ot_sector_begin': 0.5},
             [dict(s, name=s['name'].replace('Sector', 'Overtaking_sector'))
              for s in sector_block(wpnts, starts, dict(ot_flag=args.ot_flag))],
             ['start', 'end', 'ot_flag'])

    print(f'{args.name}: {len(wpnts)} waypoints, {wpnts[-1]["s_m"]:.2f} m')
    print(f'sector starts  {starts}   (Sector0 pinned to waypoint 0, '
          f'{drift:.2f} m from its stored point)')
    if widths:
        print(f'sector widths  {widths} m')

    opp = min(wpnts, key=lambda w: abs(w['s_m'] - SIM_OPP_S))
    spawn = {f'{k}{tag}': w[f'{v}_m' if k != 'stheta' else 'psi_rad']
             for tag, w in (('', wpnts[0]), ('1', opp))
             for k, v in (('sx', 'x'), ('sy', 'y'), ('stheta', 'psi'))}
    block = [f'    {k}: {v:.4f}' for k, v in spawn.items()]
    print(f'\n# ACTIVE MAP: {args.name} -- ego s = 0.0 m, opponent s = {opp["s_m"]:.1f} m')
    print('\n'.join(block))
    if args.set_spawn:
        sim = os.path.join(os.path.dirname(root), '..', 'config', 'SIM', 'sim.yaml')
        with open(sim) as f:
            lines = f.read().splitlines()
        hit = [i for i, l in enumerate(lines) if l.startswith('    # ACTIVE MAP: ')]
        if len(hit) != 1:
            raise SystemExit(f'{sim} has {len(hit)} "# ACTIVE MAP:" lines, expected 1')
        i = hit[0]
        j = next(k for k in range(i, len(lines)) if lines[k].startswith('    stheta1:')) + 1
        lines[i:j] = [f'    # ACTIVE MAP: {args.name} -- ego s = 0.0 m, '
                      f'opponent s = {opp["s_m"]:.1f} m'] + block
        with open(sim, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        print(f'\nwrote the block into {os.path.normpath(sim)}')


if __name__ == '__main__':
    main()
