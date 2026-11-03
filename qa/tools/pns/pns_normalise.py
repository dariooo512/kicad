#!/usr/bin/env python3
"""Geometry-derived uuids and a sorted item order for a .kicad_pcb.

KiCad gives new items random uuids and saves items in uuid order; the push-and-shove router walks
items in that order, so two runs on the same copper could route differently.  usage: pns_normalise.py board...
"""
import re
import sys
import uuid
from collections import Counter

UUID = re.compile(r'\(uuid "[^"]+"\)')
KINDS = (("segment", "via", "arc"), ("zone",), ("footprint",), ("gr_line", "gr_arc", "gr_rect", "gr_circle", "gr_poly", "gr_text"))


def spans(text, kinds):
    """(start, end) of the top-level blocks of these kinds."""
    for m in re.finditer(r'\n\t\((' + "|".join(kinds) + r')\b', text):
        start, depth = m.start() + 2, 0
        for end in range(start, len(text)):
            depth += {'(': 1, ')': -1}.get(text[end], 0)
            if depth == 0:
                break
        yield start, end + 1


def renamed(block):
    """(sort key, block with derived uuids)."""
    ref = re.search(r'\(property "Reference" "([^"]+)"', block) if block.startswith("(footprint") else None
    # a zone's fill comes from the last refill and may differ by a vertex: not part of its identity
    body = re.sub(r'\(filled_polygon.*', '', block, flags=re.S) if block.startswith('(zone') else block
    key = ref.group(1) if ref else re.sub(r'\s+', ' ', UUID.sub('', body))
    count = iter(range(1 << 30))
    return key, UUID.sub(lambda m: f'(uuid "{uuid.uuid5(uuid.NAMESPACE_URL, f"{key}#{next(count)}")}")', block)


def reorder(text, kinds):
    found = list(spans(text, kinds))
    if not found:
        return text
    seen, blocks = Counter(), []
    for a, b in found:
        key, block = renamed(text[a:b])
        seen[key] += 1
        if seen[key] > 1:  # identical copper twice: the second copy needs its own uuids
            key, block = renamed(UUID.sub(f'(uuid "{uuid.uuid5(uuid.NAMESPACE_URL, f"{key}@{seen[key]}")}")', text[a:b]))
            key = f"{key}@{seen[key]}"
        blocks.append((key, block))
    blocks.sort(key=lambda kb: kb[0])
    gaps = ''.join(text[b:a2 - 2] for (_, b), (a2, _) in zip(found, found[1:]))  # other items between them
    return text[:found[0][0]] + '\n\t'.join(blk for _, blk in blocks) + gaps + text[found[-1][1]:]


def normalise(pcb):
    text = open(pcb, encoding='utf-8', newline='').read()
    for kinds in KINDS:
        text = reorder(text, kinds)
    open(pcb, 'w', encoding='utf-8', newline='').write(text)




if __name__ == '__main__':
    for p in sys.argv[1:]:
        normalise(p)
