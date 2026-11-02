#!/usr/bin/env python3
"""Grid A* planner producing waypoint jobs for pns_headless_route.

usage: pns_plan.py geometry.json drc.json jobs.json [--pitch 0.05] [--only NET,...]
                   [--signal-layers F.Cu,In2.Cu,...] [--rail-layer In4.Cu --rail-nets REGEX]

geometry.json comes from `pns_headless_route --dump`, drc.json from `kicad-cli pcb drc --format json`.
Each unconnected gap becomes a job with up to three alternative waypoint plans.  The search runs
on the signal layers only, sizes tracks and vias from the net class, keeps clearance from
locked copper, pads, holes, keepouts and the board edge, and may cross unlocked foreign copper at
a cost (the router shoves it aside).
"""
import argparse
import json
import math
import re
import sys
import time

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

try:
    from numba import njit
except ImportError:  # pragma: no cover - pure Python fallback is just slow
    def njit(*args, **kwargs):
        return (lambda f: f) if not args or not callable(args[0]) else args[0]

import heapq

# Default layer roles (RocketKVM v2, 8 layers): F sig / In1 GND / In2 sig / In3 GND /
# In4 +3V3 plane / In5 sig / In6 GND / B sig.  Plane layers are only offered to their rail nets.
SIGNAL_LAYERS = ['F.Cu', 'In2.Cu', 'In5.Cu', 'B.Cu']
RAIL_LAYER = 'In4.Cu'
RAIL_NET = re.compile(r'^\+3V3')
OUTER_LAYERS = {'F.Cu', 'B.Cu'}
SHOVED_WIDTH = 0.15
SOFT_VIA_PUSH = math.inf  # mm a new via may push unlocked copper (router decides; 0.1 plans fewer, not better)
# Smaller via allowed only inside these rule areas (rule bga_fanout_vias in rocketkvm-v2.kicad_dru)
SMALL_VIA = (0.3, 0.15)
SMALL_VIA_AREAS = ['ESCAPE_TC', 'ESCAPE_P4']  # widest unlocked signal track we expect to push aside

ITEM_RE = re.compile(r'^(Pad|Track|Via|Arc)\b.*?\[(?P<net>[^\]]+)\].*? on (?P<layers>[^,]+?)(,|$)')

DIRS = np.array([(1, 0), (1, -1), (0, -1), (-1, -1), (-1, 0), (-1, 1), (0, 1), (1, 1)], dtype=np.int64)


def dist_to(mask, pitch):
    """Distance (mm) from each cell to the nearest set cell of @mask."""
    if not mask.any():
        return np.full(mask.shape, np.inf)
    return ndimage.distance_transform_edt(~mask) * pitch


PIN_STUB_MM = 1.0


def pinned_stubs(geom):
    """Indices of short tracks tied to a pad or via at one end: shoving cannot move them."""
    anchors = {}
    for p in geom['pads']:
        for layer, poly in p['polys'].items():
            for ring in poly['outlines']:
                xs = [q[0] for q in ring]
                ys = [q[1] for q in ring]
                anchors.setdefault((p['net'], layer), []).append(('ring', ring, min(xs), min(ys), max(xs), max(ys)))
    for v in geom['vias']:
        for layer in v['layers']:
            anchors.setdefault((v['net'], layer), []).append(('disc', v['pos'], v['diameter'] / 2))

    def anchored(net, layer, pt):
        for a in anchors.get((net, layer), []):
            if a[0] == 'disc':
                if math.dist(a[1], pt) <= a[2]:
                    return True
            elif a[2] <= pt[0] <= a[4] and a[3] <= pt[1] <= a[5] and point_in_ring(pt, a[1]):
                return True
        return False

    out = set()
    for k, t in enumerate(geom['tracks']):
        if t['locked'] or math.dist(t['a'], t['b']) >= PIN_STUB_MM:
            continue
        if anchored(t['net'], t['layer'], t['a']) or anchored(t['net'], t['layer'], t['b']):
            out.add(k)
    return out


class Grid:
    def __init__(self, geom, pitch):
        self.pitch = pitch
        rings = geom['outline']['outlines']
        xs = [p[0] for r in rings for p in r]
        ys = [p[1] for r in rings for p in r]
        self.x0, self.y0 = min(xs) - 0.5, min(ys) - 0.5
        self.w = int(math.ceil((max(xs) + 0.5 - self.x0) / pitch)) + 1
        self.h = int(math.ceil((max(ys) + 0.5 - self.y0) / pitch)) + 1

    def px(self, p):
        return ((p[0] - self.x0) / self.pitch, (p[1] - self.y0) / self.pitch)

    def cell(self, p):
        x, y = self.px(p)
        return int(round(y)), int(round(x))

    def mm(self, y, x):
        return (round(self.x0 + x * self.pitch, 4), round(self.y0 + y * self.pitch, 4))


def capsule(grid, a, b, width):
    """Polygon (in px) of a track segment with round ends."""
    ax, ay = grid.px(a)
    bx, by = grid.px(b)
    r = width / 2 / grid.pitch
    ang = math.atan2(by - ay, bx - ax)
    pts = []
    for cx, cy, start in ((bx, by, ang - math.pi / 2), (ax, ay, ang + math.pi / 2)):
        for k in range(9):
            t = start + math.pi * k / 8
            pts.append((cx + r * math.cos(t), cy + r * math.sin(t)))
    return pts


def disc(grid, c, d):
    cx, cy = grid.px(c)
    r = d / 2 / grid.pitch
    return [(cx + r * math.cos(2 * math.pi * k / 16), cy + r * math.sin(2 * math.pi * k / 16)) for k in range(16)]


class Board:
    def __init__(self, geom, pitch, locked_soft=False, shove_pinned=False):
        self.geom = geom
        self.grid = g = Grid(geom, pitch)
        self.layers = geom['layers']
        self.li = {name: i for i, name in enumerate(self.layers)}
        self.netnames = [''] + sorted(geom['nets'])
        self.netid = {n: i for i, n in enumerate(self.netnames)}
        self.rules = geom['rules']

        n = len(self.layers)
        items = []  # (layer, polygon px, net, hard, is_via, clearance)
        pinned = set() if shove_pinned else pinned_stubs(geom)
        for k, t in enumerate(geom['tracks']):
            items.append((t['layer'], capsule(g, t['a'], t['b'], t['width']), t['net'],
                          (t['locked'] or k in pinned) and not locked_soft, False, t.get('clearance', 0)))
        for v in geom['vias']:
            for layer in v['layers']:
                items.append((layer, disc(g, v['pos'], v['diameter']), v['net'],
                              v['locked'] and not locked_soft, True, v.get('clearance', 0)))
        for p in geom['pads']:
            for layer, poly in p['polys'].items():
                for ring in poly['outlines']:
                    items.append((layer, [g.px(q) for q in ring], p['net'], True, False, p.get('clearance', 0)))
        for s_ in geom['shapes']:
            for ring in s_['poly']['outlines']:
                items.append((s_['layer'], [g.px(q) for q in ring], s_['net'], True, False, s_.get('clearance', 0)))

        # Obstacles are grouped by their rule-resolved clearance so each group keeps its own spacing.
        self.buckets = sorted({round(it[5], 3) for it in items})
        self.owner, self.hard, self.foreign_via = [], [], []
        for bucket in self.buckets:
            owner = [Image.new('I', (g.w, g.h), 0) for _ in range(n)]
            hard = [Image.new('L', (g.w, g.h), 0) for _ in range(n)]
            via_img = [Image.new('L', (g.w, g.h), 0) for _ in range(n)]
            od = [ImageDraw.Draw(im) for im in owner]
            hd_ = [ImageDraw.Draw(im) for im in hard]
            vd_ = [ImageDraw.Draw(im) for im in via_img]
            for layer, poly, net, is_hard, is_via, clr in items:
                if round(clr, 3) != bucket or layer not in self.li:
                    continue
                i = self.li[layer]
                # Net-less copper gets an id no real net has, so it blocks every net.
                od[i].polygon(poly, fill=self.netid.get(net, 0) or 1_000_000)
                if is_hard:
                    hd_[i].polygon(poly, fill=1)
                if is_via:
                    vd_[i].polygon(poly, fill=1)
            self.owner.append(np.stack([np.asarray(im, dtype=np.int32) for im in owner]))
            self.hard.append(np.stack([np.asarray(im, dtype=bool) for im in hard]))
            self.foreign_via.append(np.stack([np.asarray(im, dtype=bool) for im in via_img]))

        holes = Image.new('I', (g.w, g.h), 0)
        hd = ImageDraw.Draw(holes)
        for p in geom['pads']:
            if 'hole' in p:
                h = p['hole']
                hd.polygon(capsule(g, h['a'], h['b'], h['width']), fill=self.netid.get(p['net'], 0) or 1_000_000)
        for v in geom['vias']:
            hd.polygon(disc(g, v['pos'], v['drill']), fill=self.netid.get(v['net'], 0) or 1_000_000)
        self.hole_owner = np.asarray(holes, dtype=np.int32)
        self.dist_hole = ndimage.distance_transform_edt(self.hole_owner == 0) * pitch

        inside = Image.new('L', (g.w, g.h), 0)
        idr = ImageDraw.Draw(inside)
        for ring in geom['outline']['outlines']:
            idr.polygon([g.px(q) for q in ring], fill=1)
        for ring in geom['outline']['holes']:
            idr.polygon([g.px(q) for q in ring], fill=0)
        # The polygon fill includes boundary pixels and the EDT is pixel-quantised: stay on the safe side.
        self.dist_edge = (ndimage.distance_transform_edt(np.asarray(inside, dtype=bool)) * pitch
                          - 1.5 * pitch - self.rules.get('edge_cut_width', 0) / 2)

        keep_t = [Image.new('L', (g.w, g.h), 0) for _ in range(n)]
        keep_v = Image.new('L', (g.w, g.h), 0)
        kv = ImageDraw.Draw(keep_v)
        self.areas = {}
        for a in geom['areas']:
            rings = [[g.px(q) for q in ring] for ring in a['poly']['outlines']]
            self.areas.setdefault(a['name'], []).extend(rings)
            for ring in rings:
                if a['no_tracks']:
                    for layer in a['layers']:
                        ImageDraw.Draw(keep_t[self.li[layer]]).polygon(ring, fill=1)
                if a['no_vias']:
                    kv.polygon(ring, fill=1)
        # A keepout edge on a pixel boundary may not be filled, so assume it is a pixel closer.
        self.dist_keep_t = np.stack([dist_to(np.asarray(im, dtype=bool), pitch) - pitch for im in keep_t])
        self.dist_keep_v = dist_to(np.asarray(keep_v, dtype=bool), pitch) - pitch

        small = Image.new('L', (g.w, g.h), 0)
        for area in SMALL_VIA_AREAS:
            for ring in self.areas.get(area, []):
                ImageDraw.Draw(small).polygon(ring, fill=1)
        # The whole via must sit inside the area for the rule to apply.
        self.dist_small_area = ndimage.distance_transform_edt(np.asarray(small, dtype=bool)) * pitch - pitch

    def net_rasters(self, net, box, safety, via_push=SOFT_VIA_PUSH):
        """Per-layer track state (0 blocked, 1 free, 2 crosses unlocked copper) and via state."""
        y0, y1, x0, x1 = box
        nid = self.netid[net]
        info = self.geom['nets'][net]
        clr = max(info['clearance'], self.rules['min_clearance'])
        w, vd, drill = info['track'], info['via_diameter'], info['via_drill']
        p = self.grid.pitch

        d_edge = self.dist_edge[y0:y1, x0:x1]
        d_hole = self.dist_hole[y0:y1, x0:x1]
        L = len(self.layers)
        hard_ok = np.ones((L, y1 - y0, x1 - x0), dtype=bool)
        # Crossing unlocked copper only helps where the shoved track has somewhere to go.
        shove_room = np.ones_like(hard_ok)
        soft_hit = np.zeros_like(hard_ok)
        d_hard_min = np.full((y1 - y0, x1 - x0), np.inf)
        d_fvia_min = np.full_like(d_hard_min, np.inf)
        d_soft_min = np.full_like(d_hard_min, np.inf)
        for k, bucket in enumerate(self.buckets):
            c = max(clr, bucket)
            owner = self.owner[k][:, y0:y1, x0:x1]
            hard = self.hard[k][:, y0:y1, x0:x1]
            foreign = (owner != 0) & (owner != nid)
            if not foreign.any():
                continue
            d_hard = np.stack([dist_to(m, p) for m in foreign & hard])
            d_soft = np.stack([dist_to(m, p) for m in foreign & ~hard])
            d_fvia = np.stack([dist_to(m, p) for m in foreign & self.foreign_via[k][:, y0:y1, x0:x1]])
            hard_ok &= d_hard >= c + w / 2 + safety
            shove_room &= d_hard >= 2 * c + 1.5 * w + SHOVED_WIDTH + safety
            soft_hit |= d_soft < c + w / 2 + safety
            # Store distances less the bucket clearance so one threshold serves every via size.
            d_hard_min = np.minimum(d_hard_min, d_hard.min(axis=0) - c)
            d_fvia_min = np.minimum(d_fvia_min, d_fvia.min(axis=0) - c)
            d_soft_min = np.minimum(d_soft_min, d_soft.min(axis=0) - c)
        holes = self.hole_owner[y0:y1, x0:x1]
        d_fhole = dist_to((holes != 0) & (holes != nid), p)

        track = (hard_ok & (self.dist_keep_t[:, y0:y1, x0:x1] >= w / 2 + safety)
                 & (d_edge >= self.rules['edge_clearance'] + w / 2 + safety)[None]
                 & (d_fhole >= self.rules['hole_clearance'] + w / 2 + safety)[None]).astype(np.uint8)
        track[(track == 1) & soft_hit] = 2
        track[(track == 2) & ~shove_room] = 0

        def via_sites(diameter, hole):
            need = diameter / 2 + safety
            ok = ((d_hard_min >= need) & (d_fvia_min >= need)
                  & (self.dist_keep_v[y0:y1, x0:x1] >= need)
                  & (d_edge >= self.rules['edge_clearance'] + need)
                  & (d_fhole >= self.rules['hole_clearance'] + need)
                  & (d_hole >= self.rules['hole_to_hole'] + hole / 2 + safety))
            sites = ok.astype(np.uint8)
            sites[ok & (d_soft_min < need)] = 2
            # A via only nudges unlocked copper; landing on it needs a push shoving rarely achieves.
            sites[ok & (d_soft_min < need - via_push)] = 0
            return sites

        via = via_sites(vd, drill)
        small = via_sites(*SMALL_VIA)
        small[self.dist_small_area[y0:y1, x0:x1] < SMALL_VIA[0] / 2] = 0
        # 0/1/2 as for tracks; 3/4 mean only the small via fits (free / over unlocked copper).
        use_small = (via == 0) & (small > 0)
        via[use_small] = small[use_small] + 2
        return track, via


@njit(cache=True)
def astar(track, via, allowed, lcost, starts, goals, soft_pen, t45, t90, via_cost, via_soft):
    L, H, W = track.shape
    n = L * H * W * 9
    g = np.full(n, np.inf, dtype=np.float32)
    parent = np.full(n, -1, dtype=np.int64)
    closed = np.zeros(n, dtype=np.uint8)
    goal = np.zeros(L * H * W, dtype=np.uint8)
    gy = goals[0, 1]
    gx = goals[0, 2]
    for k in range(goals.shape[0]):
        goal[(goals[k, 0] * H + goals[k, 1]) * W + goals[k, 2]] = 1
    dx = np.array([1, 1, 0, -1, -1, -1, 0, 1])
    dy = np.array([0, -1, -1, -1, 0, 1, 1, 1])
    heap = [(np.float32(0.0), np.int64(0))]
    heap.pop()
    for k in range(starts.shape[0]):
        s = ((starts[k, 0] * H + starts[k, 1]) * W + starts[k, 2]) * 9 + 8
        g[s] = 0.0
        heapq.heappush(heap, (np.float32(0.0), np.int64(s)))
    while len(heap) > 0:
        f, s = heapq.heappop(heap)
        if closed[s]:
            continue
        closed[s] = 1
        d = s % 9
        c = s // 9
        x = c % W
        y = (c // W) % H
        l = c // (W * H)
        if goal[c]:
            out = []
            while s >= 0:
                out.append(s)
                s = parent[s]
            return out[::-1]
        gs = g[s]
        for nd in range(8):
            if d < 8:
                k = abs(nd - d)
                k = min(k, 8 - k)
                if k > 2:
                    continue
                tc = 0.0 if k == 0 else (t45 if k == 1 else t90)
            else:
                tc = 0.0
            nx = x + dx[nd]
            ny = y + dy[nd]
            if nx < 0 or ny < 0 or nx >= W or ny >= H:
                continue
            st = track[l, ny, nx]
            nc = (l * H + ny) * W + nx
            if st == 0 and not goal[nc]:
                continue
            step = (1.4142135 if (dx[nd] != 0 and dy[nd] != 0) else 1.0) * lcost[l]
            cost = gs + step + tc + (soft_pen if st == 2 else 0.0)
            ns = nc * 9 + nd
            if cost < g[ns]:
                g[ns] = cost
                parent[ns] = s
                ax = abs(nx - gx)
                ay = abs(ny - gy)
                h = max(ax, ay) + 0.41421356 * min(ax, ay)
                heapq.heappush(heap, (np.float32(cost + h), np.int64(ns)))
        vs = via[y, x]
        if vs > 2:
            vs -= 2
        if vs != 0:
            for nl in range(L):
                if nl == l or not allowed[nl] or track[nl, y, x] == 0:
                    continue
                nc = (nl * H + y) * W + x
                ns = nc * 9 + 8
                cost = gs + via_cost + (via_soft if vs == 2 else 0.0)
                if cost < g[ns]:
                    g[ns] = cost
                    parent[ns] = s
                    ax = abs(x - gx)
                    ay = abs(y - gy)
                    h = max(ax, ay) + 0.41421356 * min(ax, ay)
                    heapq.heappush(heap, (np.float32(cost + h), np.int64(ns)))
    return [np.int64(-1)][:0]


def parse_item(board, item):
    m = ITEM_RE.match(item['description'])
    if not m:
        return None
    kind, net = m.group(1), m.group('net')
    pos = (item['pos']['x'], item['pos']['y'])
    geom = board.geom
    near = lambda a, b, tol=0.002: abs(a[0] - b[0]) < tol and abs(a[1] - b[1]) < tol

    if kind == 'Pad':
        pads = [p for p in geom['pads'] if p['net'] == net and near(p['pos'], pos)]
        if not pads:
            return None
        return {'net': net, 'pos': tuple(pads[0]['pos']), 'layers': list(pads[0]['polys']) or pads[0]['layers']}
    if kind == 'Via':
        vias = [v for v in geom['vias'] if v['net'] == net and near(v['pos'], pos)]
        if not vias:
            return None
        return {'net': net, 'pos': tuple(vias[0]['pos']), 'layers': vias[0]['layers']}

    layer = m.group('layers').strip()
    segs = [t for t in geom['tracks'] if t['net'] == net and t['layer'] == layer
            and (near(t['a'], pos) or near(t['b'], pos))]
    if not segs:
        return {'net': net, 'pos': pos, 'layers': [layer]}
    t = segs[0]

    def touched(pt):
        for o in geom['tracks']:
            if o is t or o['net'] != net or o['layer'] != layer:
                continue
            if math.dist(o['a'], pt) <= o['width'] / 2 or math.dist(o['b'], pt) <= o['width'] / 2:
                return True
        for v in geom['vias']:
            if v['net'] == net and math.dist(v['pos'], pt) <= v['diameter'] / 2:
                return True
        for p in geom['pads']:
            if p['net'] == net and layer in p['polys'] and math.dist(p['pos'], pt) < 1.5:
                for ring in p['polys'][layer]['outlines']:
                    if point_in_ring(pt, ring):
                        return True
        return False

    ends = [tuple(t['a']), tuple(t['b'])]
    free = [e for e in ends if not touched(e)]
    return {'net': net, 'pos': free[0] if len(free) == 1 else pos, 'layers': [layer]}


def point_in_ring(pt, ring):
    x, y = pt
    inside = False
    for i in range(len(ring)):
        (x1, y1), (x2, y2) = ring[i], ring[i - 1]
        if (y1 > y) != (y2 > y) and x < (x2 - x1) * (y - y1) / (y2 - y1) + x1:
            inside = not inside
    return inside


def path_to_waypoints(board, box, path, L, H, W, allowed_names, via):
    pts = []
    for s in path:
        c = s // 9
        x, y, l = c % W, (c // W) % H, c // (W * H)
        pts.append((l, y + box[0], x + box[2], s % 9))
    wps = []
    for i in range(1, len(pts) - 1):
        l, y, x, d = pts[i]
        nl, ny, nx, nd = pts[i + 1]
        if nl != l:
            wp = {'x': board.grid.mm(y, x)[0], 'y': board.grid.mm(y, x)[1],
                  'layer': allowed_names[nl], 'via': True}
            if via[y - box[0], x - box[2]] > 2:
                wp['via_diameter'], wp['via_drill'] = SMALL_VIA
            wps.append(wp)
        elif nd != d and d != 8:
            p = board.grid.mm(y, x)
            wps.append({'x': p[0], 'y': p[1]})
    return wps, allowed_names[pts[-1][0]]


PROFILES = [
    # name, soft penalty, 45 deg turn, 90 deg turn, via cost, soft via extra, outer layer cost, safety px,
    # how far (mm) a new via may push unlocked copper (None: --via-push)
    # Distances are measured to the nearest filled pixel centre, so the true edge can be up to
    # ~0.71 px closer: the sound profiles add that, the last one gambles on it (the router has the
    # final say, so an optimistic plan can only fail, never break a rule).
    ('balanced', 4.0, 1.0, 4.0, 30.0, 40.0, 1.25, 1.01, None),
    ('no_shove', 1e9, 1.0, 4.0, 30.0, 1e9, 1.25, 1.01, None),
    ('via_happy', 2.0, 1.0, 4.0, 12.0, 20.0, 1.0, 1.01, None),
    ('optimistic', 4.0, 1.0, 4.0, 30.0, 40.0, 1.25, 0.3, math.inf),
]


def plan_gap(board, a, b, margin_mm):
    net = a['net']
    g = board.grid
    names = list(SIGNAL_LAYERS) + ([RAIL_LAYER] if RAIL_NET.match(net) else [])
    names = [n for n in board.layers if n in names]
    ay, ax = g.cell(a['pos'])
    by, bx = g.cell(b['pos'])
    m = int(margin_mm / g.pitch)
    box = (max(0, min(ay, by) - m), min(g.h, max(ay, by) + m + 1),
           max(0, min(ax, bx) - m), min(g.w, max(ax, bx) + m + 1))
    plans, notes = [], []

    for name, soft_pen, t45, t90, via_cost, via_soft, outer, safety_px, via_push in PROFILES:
        track_all, via = board.net_rasters(net, box, safety_px * g.pitch,
                                           SOFT_VIA_PUSH if via_push is None else via_push)
        track = np.ascontiguousarray(np.stack([track_all[board.li[n]] for n in names]))
        if soft_pen > 1e8:
            track[track == 2] = 0
            via[(via == 2) | (via == 4)] = 0
        lcost = np.array([outer if n in OUTER_LAYERS else 1.0 for n in names], dtype=np.float64)
        allowed = np.ones(len(names), dtype=np.bool_)
        starts = np.array([(names.index(l), ay - box[0], ax - box[2]) for l in a['layers'] if l in names],
                          dtype=np.int64)
        goals = np.array([(names.index(l), by - box[0], bx - box[2]) for l in b['layers'] if l in names],
                         dtype=np.int64)
        if len(starts) == 0 or len(goals) == 0:
            notes.append(f'{name}: endpoint not on a signal layer')
            continue
        for l, y, x in starts:
            track[l, y, x] = max(track[l, y, x], 1)
        if name == PROFILES[0][0]:
            def around(pts):
                return [int((track[l, max(0, y - 3):y + 4, max(0, x - 3):x + 4] > 0).sum()) for l, y, x in pts]
            notes.append(f'free cells in 7x7: start {around(starts)} goal {around(goals)}')
        path = astar(track, via, allowed, lcost, starts, goals, min(soft_pen, 1e6), t45, t90, via_cost,
                     min(via_soft, 1e6))
        if len(path) == 0:
            notes.append(f'{name}: no path')
            continue
        wps, last_layer = path_to_waypoints(board, box, np.array(path), len(names), box[1] - box[0],
                                            box[3] - box[2], names, via)
        start_layer = names[(path[0] // 9) // ((box[1] - box[0]) * (box[3] - box[2]))]
        plan = {'start_layer': start_layer, 'waypoints': wps}
        if plan not in plans:
            plans.append(plan)
    return plans, notes


def main():
    global SIGNAL_LAYERS, RAIL_LAYER, RAIL_NET, SMALL_VIA_AREAS, SOFT_VIA_PUSH
    ap = argparse.ArgumentParser()
    ap.add_argument('geometry')
    ap.add_argument('drc')
    ap.add_argument('jobs')
    ap.add_argument('--pitch', type=float, default=0.05)
    ap.add_argument('--margin', type=float, default=4.0)
    ap.add_argument('--only', default='')
    ap.add_argument('--nets', default='', help='regex: plan only gaps of matching nets')
    ap.add_argument('--modes', default='shove,walkaround')
    # Diagnosis only: plans that need locked copper moved are not routable by pns_headless_route.
    ap.add_argument('--locked-soft', action='store_true')
    ap.add_argument('--shove-pinned', action='store_true',
                    help='let plans push short unlocked fan-out stubs tied to a pad (the router decides)')
    ap.add_argument('--small-via-areas', default=','.join(SMALL_VIA_AREAS))
    ap.add_argument('--via-push', type=float, default=SOFT_VIA_PUSH,
                    help='mm a planned via may push unlocked copper (last profile: unlimited)')
    ap.add_argument('--signal-layers', default=','.join(SIGNAL_LAYERS))
    ap.add_argument('--rail-layer', default=RAIL_LAYER)
    ap.add_argument('--rail-nets', default=RAIL_NET.pattern, help='regex of nets allowed on --rail-layer')
    args = ap.parse_args()

    SIGNAL_LAYERS = args.signal_layers.split(',')
    SMALL_VIA_AREAS = [a for a in args.small_via_areas.split(',') if a]
    SOFT_VIA_PUSH = args.via_push
    RAIL_LAYER = args.rail_layer
    RAIL_NET = re.compile(args.rail_nets)

    t0 = time.time()
    geom = json.load(open(args.geometry))
    board = Board(geom, args.pitch, args.locked_soft, args.shove_pinned)
    print(f'rasterised {board.grid.w}x{board.grid.h}x{len(board.layers)} in {time.time() - t0:.1f}s',
          file=sys.stderr)

    drc = json.load(open(args.drc, encoding='utf-8'))
    only = set(filter(None, args.only.split(',')))
    net_filter = re.compile(args.nets) if args.nets else None
    jobs, report = [], []
    for gap in drc['unconnected_items']:
        items = [parse_item(board, i) for i in gap['items'][:2]]
        desc = ' | '.join(i['description'] for i in gap['items'])
        if None in items or items[0]['net'] != items[1]['net'] or items[0]['net'] not in geom['nets']:
            report.append({'gap': desc, 'status': 'skipped'})
            continue
        a, b = sorted(items, key=lambda it: len(it['layers']))
        if only and a['net'] not in only:
            continue
        if net_filter and not net_filter.search(a['net']):
            continue
        t1 = time.time()
        plans, notes = plan_gap(board, a, b, args.margin)
        if not plans:
            plans, more = plan_gap(board, a, b, args.margin * 3)
            notes += more
        report.append({'gap': desc, 'plans': len(plans), 'notes': notes, 'secs': round(time.time() - t1, 1)})
        if not plans:
            continue
        jobs.append({'net': a['net'], 'start': list(a['pos']), 'start_layer': plans[0]['start_layer'],
                     'end': list(b['pos']), 'alternatives': plans, 'modes': args.modes.split(','),
                     'gap': desc})

    out = jobs
    json.dump(out, open(args.jobs, 'w'), indent=1)
    json.dump(report, sys.stdout, indent=1)
    print(f'\n{len(out)} jobs in {time.time() - t0:.1f}s', file=sys.stderr)


if __name__ == '__main__':
    main()
