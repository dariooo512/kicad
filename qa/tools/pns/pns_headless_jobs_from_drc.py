#!/usr/bin/env python3
"""Turn a kicad-cli DRC JSON report's unconnected items into pns_headless_route jobs.

usage: pns_headless_jobs_from_drc.py drc.json copper_layers jobs.json
  copper_layers: comma-separated copper stack, top to bottom (e.g. F.Cu,In1.Cu,...,B.Cu)
"""
import json
import re
import sys

ITEM_RE = re.compile(r'^(Pad|Track|Via|Arc)\b.*?\[(?P<net>[^\]]+)\].*? on (?P<layers>[^,]+?)(,|$)')


def parse_item(item, stack):
    m = ITEM_RE.match(item['description'])
    if not m:
        return None
    kind = m.group(1)
    if kind == 'Via':
        layers = stack
    else:
        layers = [m.group('layers').strip()]
        if layers[0] not in stack:
            layers = stack  # THT pad reported as e.g. "PTH"
    return {'kind': kind, 'net': m.group('net'), 'layers': layers,
            'pos': (item['pos']['x'], item['pos']['y'])}


def lerp(a, b, t):
    return (round(a[0] + (b[0] - a[0]) * t, 4), round(a[1] + (b[1] - a[1]) * t, 4))


def wp(p, layer=None, via=False):
    w = {'x': p[0], 'y': p[1]}
    if layer:
        w['layer'] = layer
    if via:
        w['via'] = True
    return w


DIRS = [(1, 0), (-1, 0), (0, 1), (0, -1), (0.7071, 0.7071), (-0.7071, 0.7071),
        (0.7071, -0.7071), (-0.7071, -0.7071)]
ESCAPE_MM = 0.5


def offset(p, d, dist=ESCAPE_MM):
    return (round(p[0] + d[0] * dist, 4), round(p[1] + d[1] * dist, 4))


def escape_plans(a, b, start_layer, end_layers):
    """Short stubs out of crowded pads before heading off, the way a fan-out is drawn by hand."""
    pa, pb = a['pos'], b['pos']
    out = []
    for ds in DIRS:
        for de in DIRS[:4]:
            s, e = offset(pa, ds), offset(pb, de)
            if start_layer in end_layers:
                out.append([wp(s), wp(e)])
            else:
                out.append([wp(s), wp(lerp(s, e, 0.5), end_layers[0], True), wp(e)])
                out.append([wp(s, end_layers[0], True), wp(e)])
    return out


def plans(a, b, start_layer, end_layers, stack):
    pa, pb = a['pos'], b['pos']
    corners = [(pb[0], pa[1]), (pa[0], pb[1])]
    out = []
    if start_layer in end_layers:
        out.append([])
        out += [[wp(c)] for c in corners]
        other = 'B.Cu' if start_layer != 'B.Cu' else 'F.Cu'
        out.append([wp(lerp(pa, pb, 0.3), other, True), wp(lerp(pa, pb, 0.7), start_layer, True)])
    else:
        target = end_layers[0]
        for t in (0.5, 0.25, 0.75):
            out.append([wp(lerp(pa, pb, t), target, True)])
        for c in corners:
            out.append([wp(c, target, True)])
            out.append([wp(c), wp(lerp(c, pb, 0.5), target, True)])
    return out + escape_plans(a, b, start_layer, end_layers)


def main():
    drc_path, stack_arg, out_path = sys.argv[1:4]
    stack = stack_arg.split(',')
    drc = json.load(open(drc_path, encoding='utf-8'))
    jobs, skipped = [], []

    for gap in drc['unconnected_items']:
        a, b = (parse_item(i, stack) for i in gap['items'][:2])
        if not a or not b or a['net'] != b['net']:
            skipped.append(' | '.join(i['description'] for i in gap['items']))
            continue
        # Start from the item with fewer layer choices; the end may be anything on the final layer.
        if len(a['layers']) > len(b['layers']):
            a, b = b, a
        common = [l for l in a['layers'] if l in b['layers']]
        start_layer = common[0] if common else a['layers'][0]
        jobs.append({'net': a['net'], 'start': list(a['pos']), 'start_layer': start_layer,
                     'end': list(b['pos']),
                     'alternatives': plans(a, b, start_layer, common or b['layers'], stack)})

    json.dump(jobs, open(out_path, 'w'), indent=1)
    print(f'{len(jobs)} jobs, {len(skipped)} skipped')
    for s in skipped:
        print('  skipped:', s)


if __name__ == '__main__':
    main()
