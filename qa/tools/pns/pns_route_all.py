#!/usr/bin/env python3
"""Route a board's unconnected gaps in rounds: DRC -> dump -> plan -> route.

usage: pns_route_all.py board.kicad_pcb workdir [--rounds 4]

A round's result is kept only if, after a zone refill, DRC shows no errors, no violation type
the original board did not have, and no extra zone-to-zone gaps (a new track can cut a pour into
islands).  Otherwise each routed job is replayed alone to find the culprits, which are dropped
and never planned again.

Environment: PNS_TOOL (pns_headless_route), KICAD_CLI (a Windows kicad-cli.exe works from WSL),
PYTHON (interpreter for pns_plan.py, with numpy, scipy, pillow and ideally numba).
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
from collections import Counter
from pathlib import Path

HERE = Path(__file__).resolve().parent
TOOL = os.environ.get('PNS_TOOL', 'pns_headless_route')
CLI = os.environ.get('KICAD_CLI', 'kicad-cli')
PYTHON = os.environ.get('PYTHON', sys.executable)


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
        res = subprocess.run([TOOL, str(self.dir / f'{src_name}.kicad_pcb'), str(jobs_path),
                              str(self.dir / f'{out_name}.kicad_pcb')], capture_output=True, text=True)
        result = json.loads(res.stdout)
        json.dump(result, open(self.dir / f'{out_name}.route.json', 'w'), indent=1)
        return result


def zone_gaps(drc):
    return sum(all(i['description'].startswith('Zone') for i in g['items']) for g in drc['unconnected_items'])


def health(drc, base_types, base_zone_gaps):
    """Why a refilled DRC report is worse than the original board, or None."""
    errors = sum(v['severity'] == 'error' for v in drc['violations'])
    new_types = {v['type'] for v in drc['violations']} - base_types
    if errors:
        return f'{errors} DRC errors'
    if new_types:
        return f'new violation types {sorted(new_types)}'
    if zone_gaps(drc) > base_zone_gaps:
        return f'zone islands {zone_gaps(drc)} > {base_zone_gaps}'
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('board')
    ap.add_argument('workdir')
    ap.add_argument('--rounds', type=int, default=4)
    args = ap.parse_args()

    wd = Workdir(args.board, args.workdir)
    wd.stage(args.board, 'r0')
    base = wd.drc('r0', refill=True)
    base_types = {v['type'] for v in base['violations']}
    base_zone = zone_gaps(base)
    print(f'round 0: {len(base["unconnected_items"])} unconnected ({base_zone} zone-zone)', flush=True)

    rejected = set()
    current, current_drc = 'r0', base
    for r in range(1, args.rounds + 1):
        geom = wd.dir / f'{current}.geom.json'
        subprocess.run([TOOL, '--dump', str(wd.dir / f'{current}.kicad_pcb'), str(geom)], check=True)
        drc_path = wd.dir / f'{current}.drc_refill.json'
        plan_jobs = wd.dir / f'{current}.plan_jobs.json'
        with open(wd.dir / f'{current}.plan.json', 'w') as report:
            subprocess.run([PYTHON, str(HERE / 'pns_plan.py'), str(geom), str(drc_path), str(plan_jobs)],
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
        problem = health(drc, base_types, base_zone)
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
                    why = health(wd.drc(trial, refill=True), base_types, base_zone)
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
            result = wd.route(current, keep, name)
            drc = wd.drc(name, refill=True)
            problem = health(drc, base_types, base_zone)
            if problem:
                print(f'round {r}: still {problem} with the remaining routes together; stopping', flush=True)
                break
            routed = [j for j, res in zip(keep, result['jobs']) if res['status'] == 'routed']

        current, current_drc = name, drc
        print(f'round {r}: routed {len(routed)}, {len(drc["unconnected_items"])} unconnected '
              f'({zone_gaps(drc)} zone-zone)', flush=True)

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
