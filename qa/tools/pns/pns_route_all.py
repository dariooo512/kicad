#!/usr/bin/env python3
"""Route a board's unconnected gaps in rounds: DRC -> dump -> plan -> route.

usage: pns_route_all.py board.kicad_pcb workdir [--rounds 4]

A round's result is kept only if, after a zone refill, DRC shows no errors, no violation type
the original board did not have, no more high-speed rule violations (skew, uncoupled length, via
count, length, pair gap) or dangling tracks/vias than before, no extra zone-to-zone gaps (a new track can cut a pour into
islands), and critical copper (see pns_headless_route --unlock-noncritical) is byte-identical.
Afterwards a stitching via is tried for each GND pour island that is cut off from the main pour,
where it overlaps the main pour on another layer and a via fits cleanly.  Otherwise each routed job is replayed alone to find the culprits, which are dropped
and never planned again.

Environment: PNS_TOOL (pns_headless_route), KICAD_CLI (a Windows kicad-cli.exe works from WSL),
PYTHON (interpreter for pns_plan.py, with numpy, scipy, pillow and ideally numba).
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
TOOL = os.environ.get('PNS_TOOL', 'pns_headless_route')
CLI = os.environ.get('KICAD_CLI', 'kicad-cli')
PYTHON = os.environ.get('PYTHON', sys.executable)
TOOL_ARGS = []

HIGH_SPEED_TYPES = {'skew_out_of_range', 'diff_pair_uncoupled_length_too_long', 'diff_pair_gap_out_of_range',
                    'too_many_vias', 'length_out_of_range'}
# Leftover stubs from a shove or a half-used plan: these counts may not grow either.
NO_GROWTH_TYPES = HIGH_SPEED_TYPES | {'track_dangling', 'via_dangling'}


def cli_path(p):
    if CLI.endswith('.exe'):
        return subprocess.run(['wslpath', '-w', str(p)], capture_output=True, text=True,
                              check=True).stdout.strip()
    return str(p)


class Workdir:
    def __init__(self, board, work):
        self.src = Path(board).resolve()
        self.dir = Path(work).resolve()
        self.dir.mkdir(parents=True, exist_ok=True)

    def stage(self, pcb, name):
        dst = self.dir / f'{name}.kicad_pcb'
        if Path(pcb).resolve() != dst:
            shutil.copyfile(pcb, dst)
        for ext in ('.kicad_pro', '.kicad_dru'):
            if self.src.with_suffix(ext).exists():
                shutil.copyfile(self.src.with_suffix(ext), self.dir / f'{name}{ext}')
        return dst

    def drc(self, name, refill):
        out = self.dir / f'{name}.drc{"_refill" if refill else ""}.json'
        cmd = [CLI, 'pcb', 'drc', '--format', 'json', '-o', cli_path(out)]
        if refill:
            cmd.append('--refill-zones')
        subprocess.run(cmd + [cli_path(self.dir / f'{name}.kicad_pcb')], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        return json.load(open(out, encoding='utf-8'))

    def route(self, src_name, jobs, out_name):
        jobs_path = self.dir / f'{out_name}.jobs.json'
        json.dump(jobs, open(jobs_path, 'w'), indent=1)
        self.stage(self.dir / f'{src_name}.kicad_pcb', out_name)
        res = subprocess.run([TOOL, *TOOL_ARGS, str(self.dir / f'{src_name}.kicad_pcb'), str(jobs_path),
                              str(self.dir / f'{out_name}.kicad_pcb')], capture_output=True, text=True)
        result = json.loads(res.stdout)
        json.dump(result, open(self.dir / f'{out_name}.route.json', 'w'), indent=1)
        return result


def zone_gaps(drc):
    return sum(all(i['description'].startswith('Zone') for i in g['items']) for g in drc['unconnected_items'])


def copper_blocks(pcb, uuids):
    """Normalised s-expressions of the tracks and vias whose uuid is in @uuids."""
    text = open(pcb, encoding='utf-8').read()
    out = {}
    for m in re.finditer(r'\n\t\((segment|via|arc)\b', text):
        start, depth = m.start() + 2, 0  # past the newline and tab, at the "("
        for end in range(start, len(text)):
            depth += {'(': 1, ')': -1}.get(text[end], 0)
            if depth == 0:
                break
        block = text[start:end + 1]
        uid = re.search(r'\(uuid "([^"]+)"\)', block)
        if uid and uid.group(1) in uuids:
            out[uid.group(1)] = re.sub(r'\s+', ' ', block)
    return out


class Baseline:
    def __init__(self, drc, pcb, geom):
        self.types = {v['type'] for v in drc['violations']}
        self.counts = Counter(v['type'] for v in drc['violations'])
        self.zone_gaps = zone_gaps(drc)
        self.critical = {x['uuid'] for k in ('tracks', 'vias') for x in geom[k] if x.get('critical')}
        self.critical_blocks = copper_blocks(pcb, self.critical)


def health(drc, base, pcb):
    """Why a refilled DRC report (and its board) is worse than the original, or None."""
    errors = sum(v['severity'] == 'error' for v in drc['violations'])
    counts = Counter(v['type'] for v in drc['violations'])
    new_types = set(counts) - base.types
    worse = {t: counts[t] for t in NO_GROWTH_TYPES if counts[t] > base.counts[t]}
    if errors:
        return f'{errors} DRC errors'
    if new_types:
        return f'new violation types {sorted(new_types)}'
    if worse:
        return f'more {worse}'
    if zone_gaps(drc) > base.zone_gaps:
        return f'zone islands {zone_gaps(drc)} > {base.zone_gaps}'
    if base.critical and copper_blocks(pcb, base.critical) != base.critical_blocks:
        return 'critical copper changed'
    return None


def zone_fills(pcb, net):
    """(layer, ring) for each filled island of @net's zones in a board saved after a refill."""
    text = open(pcb, encoding='utf-8').read()
    out = []
    for m in re.finditer(r'\n\t\(zone\b', text):
        start, depth = m.start() + 2, 0  # past the newline and tab, at the "("
        for end in range(start, len(text)):
            depth += {'(': 1, ')': -1}.get(text[end], 0)
            if depth == 0:
                break
        zone = text[start:end + 1]
        if f'(net "{net}")' not in zone:
            continue
        for fp in re.finditer(r'\(filled_polygon\s+\(layer "([^"]+)"\)\s+\(pts(.*?)\)\s*\)', zone, re.S):
            ring = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', fp.group(2))]
            if len(ring) >= 3:
                out.append((fp.group(1), ring))
    return out


def inside(pt, ring):
    x, y = pt
    hit = False
    for i in range(len(ring)):
        (x1, y1), (x2, y2) = ring[i], ring[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            hit = not hit
    return hit


def near_ring(pt, ring, reach):
    """Inside the ring, or within @reach of its edge (thermal reliefs cut the fill away around pads)."""
    if inside(pt, ring):
        return True
    x, y = pt
    for (ax, ay), (bx, by) in zip(ring, ring[1:] + ring[:1]):
        dx, dy = bx - ax, by - ay
        t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / (dx * dx + dy * dy or 1e-12)))
        if (x - ax - t * dx) ** 2 + (y - ay - t * dy) ** 2 <= reach * reach:
            return True
    return False


def island_components(fills, geom, net, reach=0.8):
    """Union islands joined through @net's vias, pads and tracks; largest group first."""
    parent = list(range(len(fills)))

    def find(i):
        while parent[i] != i:
            parent[i] = parent[parent[i]]
            i = parent[i]
        return i

    boxes = [(min(p[0] for p in r), min(p[1] for p in r), max(p[0] for p in r), max(p[1] for p in r))
             for _, r in fills]

    def join(points_layers):
        hit = [i for i, (layer, ring) in enumerate(fills)
               for pt, layers in points_layers
               if layer in layers and boxes[i][0] - reach <= pt[0] <= boxes[i][2] + reach
               and boxes[i][1] - reach <= pt[1] <= boxes[i][3] + reach and near_ring(pt, ring, reach)]
        for i in hit[1:]:
            parent[find(i)] = find(hit[0])

    for v in geom['vias']:
        if v['net'] == net:
            join([(v['pos'], set(v['layers']))])
    for p in geom['pads']:
        if p['net'] == net:
            join([(p['pos'], set(p['polys']))])
    for t in geom['tracks']:
        if t['net'] == net:
            join([(t['a'], {t['layer']}), (t['b'], {t['layer']})])

    groups = {}
    for i in range(len(fills)):
        groups.setdefault(find(i), []).append(i)
    return sorted(groups.values(), key=lambda g: -sum(polygon_area(fills[i][1]) for i in g))


def polygon_area(ring):
    return abs(sum(x1 * y2 - x2 * y1 for (x1, y1), (x2, y2) in zip(ring, ring[1:] + ring[:1]))) / 2


def stitch_candidates(fills, groups, geom_path, net, limit):
    """Via sites inside a stray island and a main-pour island on another layer, clear on every layer."""
    import numpy as np
    sys.path.insert(0, str(HERE))
    import pns_plan

    board = pns_plan.Board(json.load(open(geom_path)), 0.05)
    main = set(groups[0])
    sites = []
    for group in groups[1:]:
        for i in group:
            layer, ring = fills[i]
            xs, ys = [p[0] for p in ring], [p[1] for p in ring]
            y0, x0 = board.grid.cell((min(xs), min(ys)))
            y1, x1 = board.grid.cell((max(xs), max(ys)))
            box = (max(0, y0 - 2), min(board.grid.h, y1 + 3), max(0, x0 - 2), min(board.grid.w, x1 + 3))
            _, via = board.net_rasters(net, box, 1.01 * board.grid.pitch, 0.0)
            for cy, cx in zip(*np.nonzero(via == 1)):
                pt = board.grid.mm(cy + box[0], cx + box[2])
                if not inside(pt, ring):
                    continue
                if any(inside(pt, fills[j][1]) for j in main if fills[j][0] != layer):
                    sites.append(pt)
                    if len(sites) >= limit:
                        return sites
    return sites


def stitch_islands(wd, current, baseline, current_drc, net='GND', tries=6):
    """Try a stitching via per stray pour island of @net; keep the first that DRC accepts."""
    if zone_gaps(current_drc) == 0:
        return current, current_drc
    probe = f'{current}_fills'
    wd.stage(wd.dir / f'{current}.kicad_pcb', probe)
    subprocess.run([CLI, 'pcb', 'drc', '--refill-zones', '--save-board', '--format', 'json', '-o',
                    cli_path(wd.dir / f'{probe}.drc.json'), cli_path(wd.dir / f'{probe}.kicad_pcb')],
                   check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    geom_path = wd.dir / f'{current}.geom.json'
    subprocess.run([TOOL, *TOOL_ARGS, '--dump', str(wd.dir / f'{current}.kicad_pcb'), str(geom_path)], check=True)
    fills = zone_fills(wd.dir / f'{probe}.kicad_pcb', net)
    groups = island_components(fills, json.load(open(geom_path)), net)
    if len(groups) < 2:
        print(f'stitch: no stray {net} island found by geometry', flush=True)
        return current, current_drc
    sites = stitch_candidates(fills, groups, geom_path, net, tries)
    print(f'stitch: {len(groups) - 1} stray {net} island group(s), {len(sites)} candidate via sites', flush=True)
    for k, pt in enumerate(sites):
        name = f'{current}_stitch{k}'
        if wd.route(current, [{'net': net, 'stitch': list(pt)}], name)['routed'] != 1:
            continue
        drc = wd.drc(name, refill=True)
        why = health(drc, baseline, wd.dir / f'{name}.kicad_pcb')
        if why is None and zone_gaps(drc) < zone_gaps(current_drc):
            print(f'stitch: via at {pt} joins the island', flush=True)
            return name, drc
        print(f'stitch: via at {pt}: {why or "island still separate"}', flush=True)
    return current, current_drc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('workdir')
    ap.add_argument('--rounds', type=int, default=4)
    ap.add_argument('--unlock-noncritical', action='store_true',
                    help='let the router shove locked copper that is not critical')
    ap.add_argument('--critical-classes', default='Diff100,Diff90,SE50')
    ap.add_argument('--critical-nets', default='', help='regex of extra critical net names')
    ap.add_argument('--no-stitch', action='store_true', help='do not try stitching vias for stray GND islands')
    ap.add_argument('--plan-arg', action='append', default=[], help='extra pns_plan.py option (repeatable)')
    args = ap.parse_args()

    TOOL_ARGS.extend([f'--critical-classes={args.critical_classes}'])
    if args.critical_nets:
        TOOL_ARGS.append(f'--critical-nets={args.critical_nets}')
    if args.unlock_noncritical:
        TOOL_ARGS.append('--unlock-noncritical')

    wd = Workdir(args.board, args.workdir)
    wd.stage(args.board, 'r0')
    base = wd.drc('r0', refill=True)
    subprocess.run([TOOL, *TOOL_ARGS, '--dump', str(wd.dir / 'r0.kicad_pcb'), str(wd.dir / 'r0.geom.json')],
                   check=True)
    baseline = Baseline(base, wd.dir / 'r0.kicad_pcb', json.load(open(wd.dir / 'r0.geom.json')))
    print(f'round 0: {len(base["unconnected_items"])} unconnected ({baseline.zone_gaps} zone-zone), '
          f'{len(baseline.critical)} critical tracks/vias', flush=True)

    rejected = set()
    current, current_drc = 'r0', base
    for r in range(1, args.rounds + 1):
        geom = wd.dir / f'{current}.geom.json'
        subprocess.run([TOOL, *TOOL_ARGS, '--dump', str(wd.dir / f'{current}.kicad_pcb'), str(geom)], check=True)
        drc_path = wd.dir / f'{current}.drc_refill.json'
        plan_jobs = wd.dir / f'{current}.plan_jobs.json'
        with open(wd.dir / f'{current}.plan.json', 'w') as report:
            subprocess.run([PYTHON, str(HERE / 'pns_plan.py'), str(geom), str(drc_path), str(plan_jobs), *args.plan_arg],
                           check=True, stdout=report, stderr=subprocess.DEVNULL)
        jobs = [j for j in json.load(open(plan_jobs)) if j['gap'] not in rejected]
        if not jobs:
            print(f'round {r}: no plannable gaps left', flush=True)
            break

        name = f'r{r}'
        result = wd.route(current, jobs, name)
        routed = [j for j, res in zip(jobs, result['jobs']) if res['status'] == 'routed']
        if not routed:
            print(f'round {r}: {len(jobs)} planned, none routed', flush=True)
            break

        drc = wd.drc(name, refill=True)
        problem = health(drc, baseline, wd.dir / f'{name}.kicad_pcb')
        if problem:
            print(f'round {r}: {problem}; replaying {len(routed)} routes one by one', flush=True)
            keep = []
            for k, job in enumerate(routed):
                job = dict(job)
                # Drop the plan that did the damage and let the job try its other plans.
                for attempt in range(len(job['alternatives'])):
                    trial = f'{name}_t{k}_{attempt}'
                    res = wd.route(current, [job], trial)['jobs'][0]
                    if res['status'] != 'routed':
                        print(f'  dropping {job["gap"]}: no other plan routes', flush=True)
                        rejected.add(job['gap'])
                        break
                    why = health(wd.drc(trial, refill=True), baseline, wd.dir / f'{trial}.kicad_pcb')
                    if not why:
                        keep.append(job)
                        break
                    print(f'  {job["gap"]}: plan {res["plan"]} ({res["mode"]}) gives {why}', flush=True)
                    job['alternatives'] = [a for i, a in enumerate(job['alternatives']) if i != res['plan']]
                    if not job['alternatives']:
                        rejected.add(job['gap'])
                        break
            if not keep:
                continue
            jobs = keep
            result = wd.route(current, keep, name)
            drc = wd.drc(name, refill=True)
            problem = health(drc, baseline, wd.dir / f'{name}.kicad_pcb')
            if problem:
                # Fine alone, not together: add them one at a time and keep what stays healthy.
                print(f'round {r}: {problem} with the remaining routes together; adding them one by one', flush=True)
                accepted, drc = [], None
                for k, job in enumerate(keep):
                    trial = f'{name}_g{k}'
                    res = wd.route(current, accepted + [job], trial)
                    if res['routed'] != len(accepted) + 1:
                        continue
                    trial_drc = wd.drc(trial, refill=True)
                    if health(trial_drc, baseline, wd.dir / f'{trial}.kicad_pcb') is None:
                        accepted.append(job)
                        drc, result = trial_drc, res
                        shutil.copyfile(wd.dir / f'{trial}.kicad_pcb', wd.dir / f'{name}.kicad_pcb')
                    else:
                        print(f'  leaving out {job["gap"]}', flush=True)
                if not accepted:
                    print(f'round {r}: nothing combines cleanly; stopping', flush=True)
                    break
                jobs = keep = accepted
            routed = [j for j, res in zip(keep, result['jobs']) if res['status'] == 'routed']

        current, current_drc = name, drc
        print(f'round {r}: routed {len(routed)}, {len(drc["unconnected_items"])} unconnected '
              f'({zone_gaps(drc)} zone-zone)', flush=True)
        for j, res in zip(jobs, result['jobs']):
            if res['status'] != 'routed':
                print(f'  not routed: {j["gap"]}', flush=True)

    if not args.no_stitch:
        current, current_drc = stitch_islands(wd, current, baseline, current_drc)

    shutil.copyfile(wd.dir / f'{current}.kicad_pcb', wd.dir / 'final.kicad_pcb')
    summary = {'final': str(wd.dir / 'final.kicad_pcb'), 'from_round': current,
               'unconnected_before': len(base['unconnected_items']),
               'unconnected_after': len(current_drc['unconnected_items']),
               'violations_before': dict(Counter(v['type'] for v in base['violations'])),
               'violations_after': dict(Counter(v['type'] for v in current_drc['violations'])),
               'rejected_gaps': sorted(rejected),
               'remaining': [' | '.join(i['description'] for i in g['items'])
                             for g in current_drc['unconnected_items']]}
    json.dump(summary, open(wd.dir / 'summary.json', 'w'), indent=1)
    print(json.dumps(summary, indent=1))


if __name__ == '__main__':
    main()
