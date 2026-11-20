# Ipswich Kart Club: real surroundings for the measured layout.
#   python -I Tools/IpswichKC/extract_scenery.py <work dir>   (after extract_track.py + make_layout.py)
# * Buildings: OpenStreetMap footprints (building=*) inside the LiDAR area ((c) OpenStreetMap contributors, ODbL).
#   Heights are not in OSM: estimated by type (ASSUMPTION, listed per building).
# * Trees: canopy blobs detected on the Queensland aerial imagery (0.25 m/px): dark, green-dominant pixels, connected
#   components -> position + crown radius. (c) State of Queensland.
# Writes Content/KartData/Tracks/IpswichKartClub_scenery.json (game frame: x east, y south, metres; S/F origin).
import json, math, os, sys
import numpy as np
import xml.etree.ElementTree as ET
from PIL import Image

W = sys.argv[1]
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
OUT = os.path.join(ROOT, 'Content', 'KartData', 'Tracks')
st = json.load(open(os.path.join(W, 'stations.json')))
b, K = st['bbox_merc'], st['k']
E, N, hw = np.array(st['E']), np.array(st['N']), np.array(st['halfWidth'])
SF = 552; E0, N0 = E[SF], N[SF]
SE, SN = (b[2] - b[0]) / K, (b[3] - b[1]) / K
def merc(lat, lon):
    return lon * 20037508.34 / 180.0, math.log(math.tan((90 + lat) * math.pi / 360.0)) / (math.pi / 180.0) * 20037508.34 / 180.0
def local(lat, lon):
    mx, my = merc(lat, lon); return (mx - b[0]) / K, (my - b[1]) / K
def game(e, n): return [round(e - E0, 2), round(-(n - N0), 2)]
track = np.c_[E, N]
def track_dist(p):
    return float(np.min(np.hypot(track[:, 0] - p[0], track[:, 1] - p[1]) - hw))

# ---------------- buildings (OSM) ----------------
r = ET.parse(os.path.join(W, 'osm_map.xml')).getroot()
nodes = {n.get('id'): (float(n.get('lat')), float(n.get('lon'))) for n in r.iter('node')}
buildings = []
for w in r.iter('way'):
    t = {x.get('k'): x.get('v') for x in w.iter('tag')}
    if 'building' not in t: continue
    nd = [x.get('ref') for x in w.iter('nd')]
    if nd[0] != nd[-1] or any(i not in nodes for i in nd): continue
    pts = [local(*nodes[i]) for i in nd[:-1]]
    c = np.mean(pts, 0)
    if not (0 <= c[0] <= SE and 0 <= c[1] <= SN): continue
    area = 0.5 * abs(sum(pts[i][0] * pts[(i + 1) % len(pts)][1] - pts[(i + 1) % len(pts)][0] * pts[i][1] for i in range(len(pts))))
    kind = t.get('building'); name = t.get('name', '')
    # Height estimates (ASSUMPTION): toilets / small sheds 3.0 m, roofs (canopies) 3.5 m on posts,
    # large sheds (> 400 m2) 6.0 m, grandstand 5.0 m, timing tower 7.0 m, others 3.5 m.
    h = 3.5
    if kind in ('toilets',) or t.get('amenity') == 'toilets' or area < 25: h = 3.0
    if kind == 'roof' or t.get('amenity') == 'shelter': h = 3.5
    if area > 400: h = 6.0
    if 'Grandstand' in name: h = 5.0
    if 'Tower' in name: h = 7.0
    buildings.append({'osm': int(w.get('id')), 'name': name, 'type': kind, 'height': h, 'roofOnly': kind == 'roof' or t.get('amenity') == 'shelter',
                      'footprint': [game(e, n) for e, n in pts], 'area': round(area, 1)})
print('buildings: %d (total %.0f m2)' % (len(buildings), sum(x['area'] for x in buildings)))

# ---------------- trees (aerial canopy) ----------------
img = np.asarray(Image.open(os.path.join(W, 'ortho.jpg')).convert('RGB')).astype(np.float32) / 255.0
IH, IW = img.shape[:2]; px = SE / IW                      # metres per pixel
R, G, B = img[..., 0], img[..., 1], img[..., 2]
lum = 0.3 * R + 0.59 * G + 0.11 * B
# Tree crowns: darker than the dry grass / bare ground and green-dominant; tree SHADOWS are darker still and grey.
mx = img.max(-1); mn = img.min(-1); sat = (mx - mn) / np.maximum(mx, 1e-3)
# Eucalypt crowns here are grey-olive, not saturated green: darker than the dry grass, some saturation (asphalt and
# shadows on asphalt are grey), green >= red. Pure shadow on grass is very dark and is excluded by lum > 0.11.
canopy = (lum < 0.40) & (lum > 0.11) & (sat > 0.10) & (G >= R * 0.97) & (G > B * 1.06)
# downsample to 1 m cells (majority) to merge leaves into crowns, then label
f = max(1, int(round(1.0 / px)))
h2, w2 = IH // f, IW // f
cells = canopy[:h2 * f, :w2 * f].reshape(h2, f, w2, f).mean((1, 3)) > 0.45
from collections import deque
from matplotlib.path import Path as MplPath
loop_path = MplPath(track)
lab = np.zeros(cells.shape, np.int32); cur = 0; trees = []
for y in range(h2):
    for x in range(w2):
        if not cells[y, x] or lab[y, x]: continue
        cur += 1; q = deque([(y, x)]); lab[y, x] = cur; pix = []
        while q:
            cy, cx = q.popleft(); pix.append((cy, cx))
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = cy + dy, cx + dx
                if 0 <= ny < h2 and 0 <= nx < w2 and cells[ny, nx] and not lab[ny, nx]:
                    lab[ny, nx] = cur; q.append((ny, nx))
        a = len(pix) * (f * px) ** 2                     # m2
        if a < 6: continue                               # < ~1.4 m crown radius: shrub / noise
        pix = np.array(pix)
        cy, cx = pix.mean(0)
        e = (cx + 0.5) * f * px; n = SN - (cy + 0.5) * f * px
        rad = math.sqrt(a / math.pi)
        # big blobs are merged groups of trees (the forest strip): split into ~5 m crowns on a grid
        if rad > 6.5:
            sel = pix[::max(1, int(25 / (f * px) ** 2))]
            for py, pxx in sel:
                ee = (pxx + 0.5) * f * px; nn = SN - (py + 0.5) * f * px
                if track_dist((ee, nn)) > 5.0 and not loop_path.contains_point((ee, nn)): trees.append((ee, nn, 4.5))
            continue
        # infield (inside the circuit loop) is open grass with light towers: only very dark (shadowed) crowns
        # count there; nothing within 5 m of the asphalt (greener grass patches read as canopy).
        mlum = float(np.mean([lum[min(IH - 1, int(py * f + f / 2)), min(IW - 1, int(pxx * f + f / 2))] for py, pxx in pix]))
        inside = loop_path.contains_point((e, n))
        if track_dist((e, n)) > 5.0 and (not inside or mlum < 0.22): trees.append((e, n, rad))
# thin to >= 4 m spacing
trees.sort(key=lambda t: -t[2]); kept = []
for t in trees:
    if all((t[0] - k[0]) ** 2 + (t[1] - k[1]) ** 2 > 16 for k in kept): kept.append(t)
print('trees: %d crowns (radius %.1f..%.1f m)' % (len(kept), min(t[2] for t in kept), max(t[2] for t in kept)))
json.dump({'source': 'Buildings: OpenStreetMap (ODbL), heights estimated. Trees: canopy detected on QLD aerial imagery.',
           'buildings': buildings, 'trees': [[*game(t[0], t[1]), round(t[2], 2)] for t in kept]},
          open(os.path.join(OUT, 'IpswichKartClub_scenery.json'), 'w'), separators=(',', ':'))
# diagnostic
im = Image.open(os.path.join(W, 'ortho.jpg')).convert('RGB')
from PIL import ImageDraw
d = ImageDraw.Draw(im)
for t in kept:
    x, y = t[0] / px, (SN - t[1]) / px; rr = t[2] / px
    d.ellipse([x - rr, y - rr, x + rr, y + rr], outline=(255, 0, 255), width=2)
for bl in buildings:
    pts = [((p[0] + E0) / px, (SN - (N0 - p[1])) / px) for p in bl['footprint']]
    d.polygon(pts, outline=(0, 255, 255))
im.resize((IW // 2, IH // 2)).save(os.path.join(W, 'diag_scenery.jpg'), quality=88)
