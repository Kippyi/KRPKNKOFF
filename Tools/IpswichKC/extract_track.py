# Ipswich Kart Club (Willowbank, QLD) - track geometry extraction from public data.
#   python -I Tools/IpswichKC/extract_track.py <work dir with osm_track.json, ortho.jpg, dem.tif, bbox.txt>
#
# Sources (all public):
#   * OpenStreetMap way 176471927 (raceway, 1072 m loop) - (c) OpenStreetMap contributors, ODbL. Used as the seed line.
#   * Queensland Government aerial imagery (Basemaps/LatestStateProgram_AllUsers ImageServer), exported at 0.25 m/px.
#     (c) State of Queensland. Used to measure the asphalt edges, the real centre line, width and kerbs.
#   * Queensland Government LiDAR DTM (Elevation/QldDem ImageServer, 0.5 m, float32). (c) State of Queensland.
#     Used for centre-line elevation and cross-slope (camber), and for the surrounding terrain.
# Output (work dir): stations.json (per metre: centre, edges, width, elevation, camber, kerb flags) + diagnostic images.
import json, math, sys, os
import numpy as np
from PIL import Image, ImageDraw

W = sys.argv[1]
osm = json.load(open(os.path.join(W, 'osm_track.json')))
b = [float(v) for v in open(os.path.join(W, 'bbox.txt')).read().split()[0].split(',')]
K = float(open(os.path.join(W, 'bbox.txt')).read().split()[1])      # mercator scale factor at the site
img = np.asarray(Image.open(os.path.join(W, 'ortho.jpg')).convert('RGB')).astype(np.float32) / 255.0
dem = np.asarray(Image.open(os.path.join(W, 'dem.tif')), dtype=np.float32)
IH, IW = img.shape[:2]; DH, DW = dem.shape

def merc(lat, lon):
    return lon * 20037508.34 / 180.0, math.log(math.tan((90 + lat) * math.pi / 360.0)) / (math.pi / 180.0) * 20037508.34 / 180.0
# Local ground metres: E = east, N = north, origin = bbox south-west corner.
def to_local(mx, my): return (mx - b[0]) / K, (my - b[1]) / K
SPAN_E, SPAN_N = (b[2] - b[0]) / K, (b[3] - b[1]) / K
def img_px(E, N): return E / SPAN_E * IW, (SPAN_N - N) / SPAN_N * IH
def dem_px(E, N): return E / SPAN_E * DW, (SPAN_N - N) / SPAN_N * DH

def bilinear(a, x, y):
    x = np.clip(x, 0, a.shape[1] - 1.001); y = np.clip(y, 0, a.shape[0] - 1.001)
    x0 = np.floor(x).astype(int); y0 = np.floor(y).astype(int); fx = x - x0; fy = y - y0
    v = (a[y0, x0] * ((1 - fx) * (1 - fy))[..., None] if a.ndim == 3 else a[y0, x0] * (1 - fx) * (1 - fy))
    def g(dy, dx, w): return a[y0 + dy, x0 + dx] * (w[..., None] if a.ndim == 3 else w)
    return v + g(0, 1, fx * (1 - fy)) + g(1, 0, (1 - fx) * fy) + g(1, 1, fx * fy)

def elev(E, N):
    x, y = dem_px(np.asarray(E, float), np.asarray(N, float))
    return bilinear(dem, x, y)

# --- pixel classes (HSV-ish): asphalt = low saturation, mid/dark grey; kerb red; white paint ---
def classify(rgb):
    r, g, bl = rgb[..., 0], rgb[..., 1], rgb[..., 2]
    mx = rgb.max(-1); mn = rgb.min(-1); sat = (mx - mn) / np.maximum(mx, 1e-3)
    red = (r > 0.45) & (r > g * 1.45) & (r > bl * 1.45)
    white = (mn > 0.70)
    asphalt = (sat < 0.20) & (mx > 0.16) & (mx < 0.66) & ~red
    return asphalt, red, white

# --- seed line: OSM loop, resampled every 1 m ---
seed = np.array([to_local(*merc(la, lo)) for la, lo in osm['176471927']['latlon']])
if np.allclose(seed[0], seed[-1]): seed = seed[:-1]
def resample(P, step, closed=True):
    Q = np.vstack([P, P[:1]]) if closed else P
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(Q, axis=0), axis=1))]
    s = np.arange(0, d[-1], step)
    return np.c_[np.interp(s, d, Q[:, 0]), np.interp(s, d, Q[:, 1])], d[-1]
def smooth_closed(P, n):
    if n <= 1: return P
    k = np.ones(n) / n
    return np.c_[[np.convolve(np.r_[P[-n:, i], P[:, i], P[:n, i]], k, 'same')[n:-n] for i in range(2)]].T
def normals(P):
    T = np.roll(P, -1, 0) - np.roll(P, 1, 0); T /= np.linalg.norm(T, axis=1, keepdims=True)
    return T, np.c_[-T[:, 1], T[:, 0]]       # left normal (E,N frame)

line, L = resample(seed, 1.0)
for it in range(3):                              # refine: re-centre on the measured asphalt, 3 passes
    T, Nl = normals(smooth_closed(line, 5))
    offs = np.arange(-9.0, 9.01, 0.125)
    pts = line[:, None, :] + Nl[:, None, :] * offs[None, :, None]        # (S, O, 2)
    x, y = img_px(pts[..., 0], pts[..., 1])
    rgb = bilinear(img, x, y)
    asph, red, white = classify(rgb)
    track = asph | white | red                                           # paint and kerbs belong to the drivable strip
    c = len(offs) // 2
    left = np.zeros(len(line)); right = np.zeros(len(line))
    for i in range(len(line)):
        t = track[i].copy()
        # tolerate small gaps (shadows / stains) up to 0.5 m
        run = t.astype(int)
        j = c
        if not t[j]:                                                     # seed slightly off: nearest asphalt within 2 m
            cand = np.where(t[c - 16:c + 17])[0]
            if len(cand) == 0: left[i] = right[i] = np.nan; continue
            j = c - 16 + cand[np.argmin(abs(cand - 16))]
        gap = 0; k = j
        while k + 1 < len(t) and gap <= 4:
            k += 1; gap = 0 if t[k] else gap + 1
        hi = k - gap
        gap = 0; k = j
        while k - 1 >= 0 and gap <= 4:
            k -= 1; gap = 0 if t[k] else gap + 1
        lo = k + gap
        left[i], right[i] = offs[hi], offs[lo]
    ok = ~np.isnan(left)
    width = left - right
    # junctions (links / pit lane) widen the strip: clamp each edge to the local median +-0.6 m
    def medfilt(a, n=31):
        out = a.copy(); h = n // 2
        ext = np.r_[a[-h:], a, a[:h]]
        for i in range(len(a)): out[i] = np.nanmedian(ext[i:i + n])
        return out
    lm, rm = medfilt(np.where(ok, left, np.nan)), medfilt(np.where(ok, right, np.nan))
    left = np.where(ok & (abs(left - lm) < 0.6), left, lm); right = np.where(ok & (abs(right - rm) < 0.6), right, rm)
    centre_off = 0.5 * (left + right)
    line = line + Nl * centre_off[:, None]
    line = smooth_closed(line, 3)
    line, L = resample(line, 1.0)
    print('pass %d: length %.1f m, mean |shift| %.2f m, width median %.2f m (p10 %.2f, p90 %.2f)' % (
        it, L, np.nanmean(abs(centre_off)), np.nanmedian(width), np.nanpercentile(width, 10), np.nanpercentile(width, 90)))

# --- final measurement on the refined line: edges, kerbs, elevation, camber ---
T, Nl = normals(smooth_closed(line, 5))
half = 0.5 * (left - right)                       # from the last pass (same station count +-1: resample)
half = np.interp(np.linspace(0, 1, len(line)), np.linspace(0, 1, len(half)), half)
half = np.convolve(np.r_[half[-7:], half, half[:7]], np.ones(15) / 15, 'same')[7:-7]
# Asphalt-only edges + kerb strips: scan inward from each strip edge; pixels that are red / white paint (or bright
# concrete) before the grey asphalt starts form the kerb on that side.
asph_half = {}
kerb = {}
kerbw = {}
for side, sgn in (('left', 1), ('right', -1)):
    steps = np.arange(0.0, 2.01, 0.1)                                       # metres inward from the strip edge
    pts = line[:, None, :] + Nl[:, None, :] * (sgn * (half[:, None] - steps[None, :]))[..., None]
    x, y = img_px(pts[..., 0], pts[..., 1])
    asph, red, white = classify(bilinear(img, x, y))
    paint = red | white
    # kerb width: inward distance until 3 consecutive asphalt samples
    kw = np.zeros(len(line))
    for i in range(len(line)):
        a = asph[i]
        run = 0; j = 0
        for j in range(len(steps)):
            run = run + 1 if a[j] else 0
            if run >= 3: break
        kw[i] = steps[max(j - 2, 0)]
    redfrac = (red & (steps[None, :] < kw[:, None] + 0.05)).mean(1)
    has = (kw >= 0.4) & (paint[:, :8].mean(1) > 0.25)
    k = has.astype(int)
    for _ in range(3): k = np.maximum(k, np.maximum(np.roll(k, 1), np.roll(k, -1)))   # close 3 m gaps
    for _ in range(3): k = np.minimum(k, np.minimum(np.roll(k, 1), np.roll(k, -1)))
    for _ in range(2): k = np.minimum(k, np.minimum(np.roll(k, 1), np.roll(k, -1)))   # drop runs < 5 m
    for _ in range(2): k = np.maximum(k, np.maximum(np.roll(k, 1), np.roll(k, -1)))
    kerb[side] = k.astype(bool)
    kerbw[side] = np.where(kerb[side], np.clip(kw, 0.4, 1.6), 0.0)
    asph_half[side] = half - kerbw[side]
asphalt_width = asph_half['left'] + asph_half['right']
print('asphalt width: median %.2f m (p10 %.2f, p90 %.2f); kerb width median %.2f m' % (
    np.median(asphalt_width), np.percentile(asphalt_width, 10), np.percentile(asphalt_width, 90),
    np.median(np.r_[kerbw['left'][kerb['left']], kerbw['right'][kerb['right']]])))
zc = elev(line[:, 0], line[:, 1])
zl = elev(line[:, 0] + Nl[:, 0] * half * 0.8, line[:, 1] + Nl[:, 1] * half * 0.8)
zr = elev(line[:, 0] - Nl[:, 0] * half * 0.8, line[:, 1] - Nl[:, 1] * half * 0.8)
camber_deg = np.degrees(np.arctan2(zl - zr, 2 * half * 0.8))     # + = left edge higher
print('final: %d stations, length %.1f m, width %.2f..%.2f m (median %.2f), elevation %.2f..%.2f m, camber %.1f..%.1f deg' % (
    len(line), L, 2 * half.min(), 2 * half.max(), np.median(2 * half), zc.min(), zc.max(), camber_deg.min(), camber_deg.max()))
print('kerbs: left %.0f m, right %.0f m' % (kerb['left'].sum(), kerb['right'].sum()))

json.dump({'source': 'OSM way 176471927 (ODbL) refined on QLD aerial imagery; elevation QLD LiDAR DTM 0.5 m',
           'frame': 'local metres, E = east, N = north, origin = bbox SW corner', 'bbox_merc': b, 'k': K,
           'E': line[:, 0].round(3).tolist(), 'N': line[:, 1].round(3).tolist(), 'halfWidth': half.round(3).tolist(),
           'z': zc.round(3).tolist(), 'camberDeg': camber_deg.round(2).tolist(),
           'kerbLeft': kerb['left'].astype(int).tolist(), 'kerbRight': kerb['right'].astype(int).tolist(),
           'kerbWidthLeft': kerbw['left'].round(2).tolist(), 'kerbWidthRight': kerbw['right'].round(2).tolist(),
           'asphaltWidth': asphalt_width.round(3).tolist(), 'length': L},
          open(os.path.join(W, 'stations.json'), 'w'))

# --- diagnostics: edges + kerbs on the aerial ---
im = Image.open(os.path.join(W, 'ortho.jpg')).convert('RGB'); d = ImageDraw.Draw(im)
def poly(P, col, w=2): d.line([img_px(e, n) for e, n in P] + [img_px(*P[0])], fill=col, width=w)
poly(line, (255, 255, 0), 1)
poly(line + Nl * half[:, None], (0, 255, 255), 1); poly(line - Nl * half[:, None], (0, 255, 255), 1)
for side, sgn in (('left', 1), ('right', -1)):
    for i in np.where(kerb[side])[0]:
        e, n = line[i] + Nl[i] * sgn * (half[i] - 0.3)
        x, y = img_px(e, n); d.ellipse([x - 2, y - 2, x + 2, y + 2], fill=(255, 0, 255))
im.save(os.path.join(W, 'diag_edges.jpg'), quality=90)
