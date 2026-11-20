# Ipswich Kart Club: game layouts + LiDAR terrain from the measured stations (Tools/IpswichKC/extract_track.py).
#   python -I Tools/IpswichKC/make_layout.py <work dir>
# Writes Content/KartData/Tracks/IpswichKartClub_B.json (clockwise, championship direction),
#        Content/KartData/Tracks/IpswichKartClub_K.json (anticlockwise) and IpswichKartClub_terrain.bin.
#
# Track frame (game): x = east, y = south (UE left-handed: X forward, Y right), metres, origin = start/finish line.
# Elevations are relative to BASE (the lowest centre-line point); the terrain uses the same datum.
import json, math, os, sys, struct
import numpy as np
from PIL import Image

W = sys.argv[1]
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
OUT = os.path.join(ROOT, 'Content', 'KartData', 'Tracks')
st = json.load(open(os.path.join(W, 'stations.json')))
E, N = np.array(st['E']), np.array(st['N'])
z = np.array(st['z']); width = np.array(st['asphaltWidth'])
kL, kR = np.array(st['kerbLeft'], bool), np.array(st['kerbRight'], bool)
kw = np.r_[np.array(st['kerbWidthLeft'])[kL], np.array(st['kerbWidthRight'])[kR]]
b, K = st['bbox_merc'], st['k']
dem = np.asarray(Image.open(os.path.join(W, 'dem.tif')), dtype=np.float32)
DH, DW = dem.shape
SE, SN = (b[2] - b[0]) / K, (b[3] - b[1]) / K
S = len(E)

# Centre-line smoothing: the measured line carries pixel jitter (+-0.1 m) that reads as false corners (|k| up to
# 0.035 on the straights, R 28 m). Periodic smoothing spline, tolerance sigma 0.15 m (mean deviation 0.17 m), then a
# local minimum-radius constraint of 6.5 m: the NW hairpin's measured strip flares into run-off (11.6 m) and pulled
# the centre to an impossible R 4.3 m. Elevation / width / kerbs keep their station indices.
from scipy.interpolate import splprep, splev
_n = len(E)
_tck, _u = splprep([np.r_[E, E[0]], np.r_[N, N[0]]], s=(_n + 1) * 0.15 ** 2, per=1)
_x, _y = splev(np.linspace(0, 1, _n, endpoint=False), _tck)
_x, _y = np.array(_x), np.array(_y)
def _curv(x, y):
    xx, yy = np.r_[x[-5:], x, x[:5]], np.r_[y[-5:], y, y[:5]]
    dx, dy = np.gradient(xx), np.gradient(yy); ddx, ddy = np.gradient(dx), np.gradient(dy)
    return ((dx * ddy - dy * ddx) / np.power(dx * dx + dy * dy, 1.5))[5:-5]
for _it in range(400):
    _bad = np.where(abs(_curv(_x, _y)) > 1 / 6.5)[0]
    if len(_bad) == 0: break
    _m = np.zeros(_n, bool)
    for _i in _bad: _m[np.arange(_i - 4, _i + 5) % _n] = True
    _xs = 0.25 * np.roll(_x, 1) + 0.5 * _x + 0.25 * np.roll(_x, -1)
    _ys = 0.25 * np.roll(_y, 1) + 0.5 * _y + 0.25 * np.roll(_y, -1)
    _x, _y = np.where(_m, _xs, _x), np.where(_m, _ys, _y)
print('centre line: smoothing dev max %.2f m mean %.2f m, min radius %.1f m (%d constraint passes)' % (
    np.hypot(_x - E, _y - N).max(), np.hypot(_x - E, _y - N).mean(), 1 / abs(_curv(_x, _y)).max(), _it))
E, N = _x, _y

SF = 552                                   # start/finish: full-width white line at station 551.75 (aerial)
BASE = float(z.min())
E0, N0 = E[SF], N[SF]

def circ_median(a, n):
    h = n // 2; ext = np.r_[a[-h:], a, a[:h]]
    return np.array([np.median(ext[i:i + n]) for i in range(len(a))])
def circ_mean(a, n):
    h = n // 2; ext = np.r_[a[-h:], a, a[:h]]
    return np.convolve(ext, np.ones(n) / n, 'same')[h:-h]
# Width: junctions with the link roads / pit lane widen the measured strip; median over 21 m, clamped to the
# range of the clean sections, then lightly smoothed.
w_s = circ_mean(np.clip(circ_median(width, 21), 8.0, 10.5), 9)
z_s = circ_mean(z, 7)                       # LiDAR noise ~cm: 7 m mean keeps every real crest / dip

def dem_at(e, n):
    x = np.clip(e / SE * DW, 0, DW - 1.001); y = np.clip((SN - n) / SN * DH, 0, DH - 1.001)
    x0, y0 = int(x), int(y); fx, fy = x - x0, y - y0
    return (dem[y0, x0] * (1 - fx) * (1 - fy) + dem[y0, x0 + 1] * fx * (1 - fy) + dem[y0 + 1, x0] * (1 - fx) * fy + dem[y0 + 1, x0 + 1] * fx * fy)

def to_game(e, n): return (e - E0, -(n - N0))

def build(reverse):
    idx = list(range(SF, S)) + list(range(0, SF))          # start at the S/F line (OSM order = anticlockwise)
    if reverse:
        idx = [idx[0]] + idx[1:][::-1]
    step = 4                                                  # control point every 4 m (Catmull-Rom in game)
    sel = idx[::step]
    pts = [to_game(E[i], N[i]) for i in sel]
    n = len(pts)
    ctl = []
    for c in range(n):
        x, y = pts[c]
        # actor-frame turn sign (as AKartTestTrackActor::BuildTrack): A x B with A = P - Pprev, B = Pnext - P
        ax, ay = x - pts[c - 1][0], y - pts[c - 1][1]
        bx, by = pts[(c + 1) % n][0] - x, pts[(c + 1) % n][1] - y
        turn = ax * by - ay * bx
        tx, ty = pts[(c + 1) % n][0] - pts[c - 1][0], pts[(c + 1) % n][1] - pts[c - 1][1]
        tl = math.hypot(tx, ty); tx, ty = tx / tl, ty / tl
        lx, ly = -ty, tx                                     # actor Left(I) in game coords
        le, ln = lx, -ly                                     # same direction in east / north
        i = sel[c]; hw = 0.4 * w_s[i]
        e, nn = E[i], N[i]
        zl = np.mean([dem_at(e + le * hw + tx * d, nn - ly * 0 + ln * hw - ty * d) for d in (-1.0, 0.0, 1.0)])
        zr = np.mean([dem_at(e - le * hw + tx * d, nn - ln * hw - ty * d) for d in (-1.0, 0.0, 1.0)])
        bank = math.atan2(zl - zr, 2 * hw)                    # + raises the actor-Left edge
        cam = -math.degrees(bank) * (1.0 if turn >= 0 else -1.0)   # layout convention: + banked / - off-camber
        ctl.append([round(x, 3), round(y, 3), round(cam, 2), round(float(z_s[i] - BASE), 3), round(float(w_s[i]), 2)])
    # Kerbs: measured per station on the driver's left / right (east-north frame, OSM order). In the reversed
    # direction the driver's sides swap. Runs -> control-point index ranges.
    def runs(flags):
        out = []; c = 0
        on = [bool(any(flags[sel[k]:sel[k] + step])) if sel[k] + step <= S else bool(flags[sel[k]]) for k in range(n)]
        k = 0
        while k < n:
            if on[k]:
                j = k
                while j + 1 < n and on[j + 1]: j += 1
                out.append((k, j)); k = j + 1
            else:
                k += 1
        return out
    left_flags, right_flags = (kR, kL) if reverse else (kL, kR)
    kerbs = [{'from': a, 'to': c, 'side': 'left', 'type': 'flat'} for a, c in runs(left_flags)] + \
            [{'from': a, 'to': c, 'side': 'right', 'type': 'flat'} for a, c in runs(right_flags)]
    return ctl, kerbs

# --- terrain: LiDAR DTM resampled to the game frame (x east, y south), heights relative to BASE ---
cell = 0.5
xs = np.arange(-E0, SE - E0, cell); ys = np.arange(-(SN - N0), N0, cell)     # game x / y ranges covered by the DEM
nx, ny = len(xs), len(ys)
grid = np.empty((ny, nx), np.float32)
for r, gy in enumerate(ys):
    nrow = N0 - gy
    yy = (SN - nrow) / SN * DH
    xx = (xs + E0) / SE * DW
    x0 = np.clip(xx.astype(int), 0, DW - 2); fx = np.clip(xx - x0, 0, 1)
    y0 = int(np.clip(yy, 0, DH - 2)); fy = float(np.clip(yy - y0, 0, 1))
    row = (dem[y0, x0] * (1 - fx) + dem[y0, x0 + 1] * fx) * (1 - fy) + (dem[y0 + 1, x0] * (1 - fx) + dem[y0 + 1, x0 + 1] * fx) * fy
    grid[r] = row - BASE
terrain_file = 'IpswichKartClub_terrain.bin'
with open(os.path.join(OUT, terrain_file), 'wb') as f:
    f.write(grid.astype('<f4').tobytes())
terrain = {'file': terrain_file, 'originX': round(float(xs[0]), 3), 'originY': round(float(ys[0]), 3), 'cell': cell,
           'nx': nx, 'ny': ny, 'meshCell': 1.25, 'blend': 3.0}

meta = ('// Ipswich Kart Club, Willowbank QLD (Ipswich Motorsport Precinct, 56 Champions Way). Built from public data only:\n'
        '//  * centre line seeded from OpenStreetMap way 176471927 ((c) OpenStreetMap contributors, ODbL), refined to the\n'
        '//    asphalt edges measured on Queensland Government aerial imagery (LatestStateProgram, 0.25 m/px);\n'
        '//  * elevation, camber and the surrounding terrain from the Queensland Government LiDAR DTM (QldDem, 0.5 m);\n'
        '//    (c) State of Queensland. Kerbs (red/white, %.2f m wide) and width detected on the same imagery.\n'
        '//  * direction / names from the club lap-record sheets (B Track clockwise, K Track anticlockwise, 2026).\n'
        '// Generated by Tools/IpswichKC/make_layout.py - do not hand-edit (re-run the pipeline instead).\n'
        '// ASSUMPTIONS: both directions use the full 1.09 km outer loop (the infield link roads are not raced);\n'
        '// widths at junctions are interpolated from the neighbouring clean sections (8.0 - 10.5 m).\n'
        '// Control points: [x east, y south, camber_deg, elevation_m, width_m]; kerbs side = driver\'s left / right.\n') % float(np.median(kw))
for name, rev, label in (('IpswichKartClub_B.json', True, 'Ipswich Kart Club - B Track (clockwise)'),
                         ('IpswichKartClub_K.json', False, 'Ipswich Kart Club - K Track (anticlockwise)')):
    ctl, kerbs = build(rev)
    body = {'name': label, 'width': round(float(np.percentile(w_s, 10)), 2), 'kerbRadius': 40.0,
            'kerbWidth': round(float(np.median(kw)), 2), 'terrain': terrain, 'scenery': 'IpswichKartClub_scenery.json', 'controlPoints': ctl, 'kerbs': kerbs}
    txt = meta + json.dumps(body, separators=(',', ':')).replace('],[', '],\n[')
    open(os.path.join(OUT, name), 'w').write('{\n' + txt[txt.index('{') + 1:] if False else meta + json.dumps(body, indent=None, separators=(',', ':')))
    L = sum(math.dist(ctl[i][:2], ctl[(i + 1) % len(ctl)][:2]) for i in range(len(ctl)))
    print('%s: %d control points, polyline %.1f m, %d kerb runs, elevation 0..%.2f m, camber %.1f..%.1f deg, width %.2f..%.2f m' % (
        name, len(ctl), L, len(kerbs), max(c[3] for c in ctl), min(c[2] for c in ctl), max(c[2] for c in ctl),
        min(c[4] for c in ctl), max(c[4] for c in ctl)))
print('terrain %s: %d x %d @ %.1f m, origin (%.1f, %.1f), heights %.2f..%.2f m rel. base %.2f m AHD' % (
    terrain_file, nx, ny, cell, xs[0], ys[0], grid.min(), grid.max(), BASE))
