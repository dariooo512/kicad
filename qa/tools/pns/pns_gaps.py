#!/usr/bin/env python3
"""Unconnected gaps computed from the board geometry, independent of which endpoints DRC reports.

Per net, copper (pads, vias, track ends, refilled zone islands) is split into connected clusters;
the clusters are joined by a minimum spanning tree over the closest anchor pairs (pad and via
centres, track ends), ties broken by coordinates, so the same board always gives the same gaps
whatever the item order.  kicad-cli DRC's ratsnest picks between equally short links at random.

A net whose cluster count disagrees with DRC's gap count falls back to DRC's gaps, with each
endpoint replaced by the closest anchor pair between the two clusters DRC names.
"""
import math
import re
import sys
from collections import defaultdict

from shapely import STRtree
from shapely.geometry import LineString, Point, Polygon

EPS = 0.001
KIND_ORDER = {'pad': 0, 'via': 1, 'track': 2}
DRC_ITEM = re.compile(r'^(Pad|Track|Via|Arc|Zone)\b.*?\[(?P<net>[^\]]+)\]')


def zone_fills(pcb):
    """(net, layer, polygon) of every filled zone island in a board saved after a refill."""
    text = open(pcb, encoding='utf-8').read()
    out = []
    for m in re.finditer(r'\n\t\(zone\b', text):
        start, depth = m.start() + 2, 0
        for end in range(start, len(text)):
            depth += {'(': 1, ')': -1}.get(text[end], 0)
            if depth == 0:
                break
        zone = text[start:end + 1]
        net = re.search(r'\(net "([^"]*)"\)', zone)
        if not net or not net.group(1):
            continue
        for fp in re.finditer(r'\(filled_polygon\s+\(layer "([^"]+)"\)\s+\(pts(.*?)\)\s*\)', zone, re.S):
            ring = [(float(x), float(y)) for x, y in re.findall(r'\(xy ([-\d.]+) ([-\d.]+)\)', fp.group(2))]
            if len(ring) >= 3:
                poly = Polygon(ring)
                out.append((net.group(1), fp.group(1), poly if poly.is_valid else poly.buffer(0)))
    return out


def rkey(p):
    return (round(p[0], 4), round(p[1], 4))


class Net:
    """Copper of one net: shapes per layer, anchors, union-find over items."""

    def __init__(self, name):
        self.name = name
        self.items = []  # (kind, pos, layers, {layer: shape})
        self.parent = []

    def add(self, kind, pos, layers, shapes):
        self.items.append((kind, tuple(pos) if pos else None, tuple(layers), shapes))
        self.parent.append(len(self.parent))

    def find(self, i):
        while self.parent[i] != i:
            self.parent[i] = self.parent[self.parent[i]]
            i = self.parent[i]
        return i

    def union(self, i, j):
        a, b = self.find(i), self.find(j)
        if a != b:
            self.parent[max(a, b)] = min(a, b)


def build_nets(geom, fills):
    out = {}

    def net(name):
        if name not in out:
            out[name] = Net(name)
        return out[name]

    for p in geom['pads']:
        if not p['net']:
            continue
        shapes = {}
        for layer, poly in p['polys'].items():
            parts = [Polygon(r) for r in poly['outlines'] if len(r) >= 3]
            if parts:
                u = parts[0]
                for q in parts[1:]:
                    u = u.union(q)
                shapes[layer] = u if u.is_valid else u.buffer(0)
        net(p['net']).add('pad', p['pos'], list(p['polys']) or p['layers'], shapes)
    for v in geom['vias']:
        if v['net']:
            d = Point(v['pos']).buffer(v['diameter'] / 2, 16)
            net(v['net']).add('via', v['pos'], v['layers'], {l: d for l in v['layers']})
    for t in geom['tracks']:
        if t['net']:
            a, b = tuple(t['a']), tuple(t['b'])
            line = LineString([a, b]) if a != b else Point(a)
            net(t['net']).add('track', None, [t['layer']], {t['layer']: line.buffer(t['width'] / 2, 8)})
            n = out[t['net']]
            n.items[-1] = ('track', (a, b), (t['layer'],), n.items[-1][3])
    for name, layer, poly in fills:
        net(name).add('zone', None, [layer], {layer: poly})
    return out


def anchors_of(item):
    kind, pos, layers, _ = item
    if kind == 'track':
        return [pos[0], pos[1]]
    if kind == 'zone':
        return []
    return [pos]


def connect(n):
    """Union items that KiCad's connectivity would join (same layer)."""
    by_layer = defaultdict(list)
    for i, (_, _, _, shapes) in enumerate(n.items):
        for layer, s in shapes.items():
            by_layer[layer].append((i, s))
    for layer, entries in by_layer.items():
        shapes = [s for _, s in entries]
        tree = STRtree(shapes)
        for i, s in entries:
            for j in tree.query(s.buffer(EPS)):
                ii, s2 = entries[j]
                if ii > i and n.find(i) != n.find(ii) and joined(n.items[i], s, n.items[ii], s2):
                    n.union(i, ii)


def joined(x, sx, y, sy):
    """A track joins pads, vias and tracks where an anchor of one lies in the other; anything else by overlap."""
    if 'track' not in (x[0], y[0]) or 'zone' in (x[0], y[0]):
        return sx.distance(sy) <= EPS
    return (any(sy.distance(Point(a)) <= EPS for a in anchors_of(x))
            or any(sx.distance(Point(a)) <= EPS for a in anchors_of(y)))


def clusters(n):
    groups = defaultdict(list)
    for i in range(len(n.items)):
        groups[n.find(i)].append(i)
    out = []
    for members in groups.values():
        anchors = set()
        for i in members:
            kind, _, layers, _ = n.items[i]
            for a in anchors_of(n.items[i]):
                # A track end over a pad or via is that item's anchor: prefer the pad/via.
                anchors.add((rkey(a), KIND_ORDER[kind], layers))
        best = {}
        for pos, k, layers in sorted(anchors):
            best.setdefault((pos, layers), (pos, k, layers))
        out.append({'members': members, 'anchors': sorted(best.values())})
    # stable cluster order: by the smallest anchor (zone-only clusters last, by bounds)
    def key(c):
        if c['anchors']:
            return (0, c['anchors'][0])
        b = n.items[c['members'][0]][3]
        layer, s = sorted(b.items())[0]
        return (1, (rkey(s.bounds[:2]), 9, (layer,)))
    return sorted(out, key=key)


def closest(ca, cb):
    best = None
    for a in ca['anchors']:
        for b in cb['anchors']:
            d = round(math.dist(a[0], b[0]), 4)
            k = (d, a, b)
            if best is None or k < best:
                best = k
    return best


def mst(cl):
    anchored = [c for c in cl if c['anchors']]
    edges = []
    for i in range(len(anchored)):
        for j in range(i + 1, len(anchored)):
            d, a, b = closest(anchored[i], anchored[j])
            edges.append((d, a, b, i, j))
    edges.sort()
    parent = list(range(len(anchored)))

    def find(x):
        while parent[x] != x:
            parent[x] = parent[parent[x]]
            x = parent[x]
        return x

    out = []
    for d, a, b, i, j in edges:
        ri, rj = find(i), find(j)
        if ri != rj:
            parent[max(ri, rj)] = min(ri, rj)
            out.append((a, b))
    return out


def endpoint(net, anchor):
    pos, kind, layers = anchor
    return {'net': net, 'pos': pos, 'layers': list(layers),
            'kind': [k for k, v in KIND_ORDER.items() if v == kind][0]}


def describe(e):
    return f'{e["kind"].capitalize()} [{e["net"]}] at ({e["pos"][0]:.4f}, {e["pos"][1]:.4f}) on {",".join(e["layers"])}'


def drc_counts(drc):
    out = defaultdict(int)
    for g in drc['unconnected_items']:
        m = DRC_ITEM.match(g['items'][0]['description'])
        if m:
            out[m.group('net')] += 1
    return out


def cluster_of(n, cl, item):
    """Index of the cluster holding a DRC-reported item (matched by its anchor position)."""
    pos = (item['pos']['x'], item['pos']['y'])
    best = None
    for k, c in enumerate(cl):
        for a in c['anchors']:
            d = math.dist(a[0], pos)
            if best is None or d < best[0]:
                best = (d, k)
    return best[1] if best and best[0] < 0.01 else None


def gaps(geom, fills, drc, log=sys.stderr):
    """[(a, b)] endpoint pairs, one per gap that has an anchor on both sides, in a stable order."""
    nets = build_nets(geom, fills)
    counts = drc_counts(drc)
    out = []
    for name in sorted(set(counts) | {k for k in nets}):
        n = nets.get(name)
        if n is None:
            continue
        if name not in counts:
            continue  # DRC says connected: zone connectivity we model may differ, trust it
        connect(n)
        cl = clusters(n)
        zone_only = sum(1 for c in cl if not c['anchors'])
        anchored = len(cl) - zone_only
        mine = max(0, anchored - 1) + zone_only
        if mine == counts[name]:
            for a, b in mst(cl):
                out.append((endpoint(name, a), endpoint(name, b)))
            continue
        print(f'gaps: {name}: {mine} by geometry, {counts[name]} by DRC; using DRC pairs', file=log)
        seen = set()
        for g in drc['unconnected_items']:
            m = DRC_ITEM.match(g['items'][0]['description'])
            if not m or m.group('net') != name or len(g['items']) < 2:
                continue
            ka, kb = (cluster_of(n, cl, it) for it in g['items'][:2])
            if ka is None or kb is None or ka == kb:
                continue
            pair = tuple(sorted((ka, kb)))
            if pair in seen:
                continue
            seen.add(pair)
            d, a, b = closest(cl[pair[0]], cl[pair[1]])
            out.append((endpoint(name, a), endpoint(name, b)))
    out.sort(key=lambda ab: (ab[0]['net'], ab[0]['pos'], ab[1]['pos']))
    return out


if __name__ == '__main__':
    import json
    geom = json.load(open(sys.argv[1]))
    fills = zone_fills(sys.argv[2])
    drc = json.load(open(sys.argv[3], encoding='utf-8'))
    for a, b in gaps(geom, fills, drc):
        print(describe(a), '|', describe(b))
    print(len(drc['unconnected_items']), 'DRC gaps', file=sys.stderr)
