# Cleans the AI-generated suit base colour (racing_suit_3d_model_basecolor.jpg, Tripo) into flat suit panels:
#   navy base, black side panels / gloves / boots, thin red and white piping.
# The raw map has smeared, garbled "logos" (orange blobs on the shoulders), speckle and baked-in shading, which read as
# dirty in game. Each texel is classified by colour, speckle is removed with a majority filter, and accent regions that
# are wide blobs (not piping) are returned to the base colour. Output colours are the class medians of the source.
#   python Tools/DriverSuit/clean_suit_texture.py <in basecolor> <out png>
import sys, colorsys
import numpy as np
from PIL import Image
from scipy import ndimage as ndi

src, dst = sys.argv[1], sys.argv[2]
img = np.asarray(Image.open(src).convert('RGB')).astype(np.float32) / 255.0
r, g, b = img[..., 0], img[..., 1], img[..., 2]
mx, mn = img.max(-1), img.min(-1); v = mx; s = np.where(mx > 1e-4, (mx - mn) / np.maximum(mx, 1e-4), 0)
hue = np.zeros_like(v)
d = np.maximum(mx - mn, 1e-6)
hue = np.where(mx == r, ((g - b) / d) % 6, np.where(mx == g, (b - r) / d + 2, (r - g) / d + 4)) * 60.0
BLUE, BLACK, RED, WHITE = 0, 1, 2, 3
cls = np.full(v.shape, BLUE, np.uint8)
cls[(v < 0.22) | ((s < 0.35) & (v < 0.35))] = BLACK
cls[(s < 0.22) & (v > 0.55)] = WHITE
cls[((hue < 35) | (hue > 330)) & (s > 0.45) & (v > 0.30)] = RED
# majority filter (speckle, smeared transitions)
onehot = np.stack([ndi.uniform_filter((cls == k).astype(np.float32), 9) for k in range(4)], -1)
cls = onehot.argmax(-1).astype(np.uint8)
# accents: keep thin piping, drop wide blobs (garbled logos, smears) back to the base colour
for k, rad in ((RED, 9), (WHITE, 9)):
    m = cls == k
    blob = ndi.binary_dilation(ndi.binary_opening(m, structure=np.ones((2 * rad + 1, 2 * rad + 1))), iterations=3)
    cls[m & blob] = BLUE
    print('class %d: %.1f%% kept as piping, %.1f%% blobs removed' % (k, 100 * (m & ~blob).mean(), 100 * (m & blob).mean()))
# connected components: piping is long (keep accent pieces >= 160 px long, 4 % of the map); short accent fragments,
# small black specks and small base-colour holes inside black panels are smear -> merged into their surroundings
for k, keep in ((RED, 160), (WHITE, 160)):
    lab, n = ndi.label(cls == k)
    sl = ndi.find_objects(lab)
    for i, s_ in enumerate(sl, 1):
        if max(s_[0].stop - s_[0].start, s_[1].stop - s_[1].start) < keep: cls[s_][lab[s_] == i] = BLUE
for k, other, min_area in ((BLACK, BLUE, 6000), (BLUE, BLACK, 6000)):
    lab, n = ndi.label(cls == k)
    area = np.bincount(lab.ravel())
    small = area < min_area; small[0] = False
    cls[small[lab]] = other
print('after components: shares', np.round(np.bincount(cls.ravel(), minlength=4) / cls.size, 4))
src_cls = cls
pal = {}
for k, fallback in ((BLUE, (0.10, 0.22, 0.48)), (BLACK, (0.05, 0.05, 0.06)), (RED, (0.75, 0.16, 0.08)), (WHITE, (0.88, 0.88, 0.90))):
    m = src_cls == k
    pal[k] = np.median(img[m], 0) if m.sum() > 1000 else np.array(fallback)
# white piping: at least 0.85 (the source median is darkened by the baked shading)
pal[WHITE] = np.maximum(pal[WHITE], 0.85)
# base: the source median (sRGB ~0.24/0.38/0.56) washes out to baby blue under the physical sun; deeper racing blue
pal[BLUE] = np.array([0.10, 0.22, 0.52])
# the source medians of the small accent / black classes are muddied by smear and shading: clean racing red, deep black
pal[RED] = np.array([0.80, 0.10, 0.06]); pal[BLACK] = np.array([0.035, 0.035, 0.04])
print('palette', {k: np.round(c, 3).tolist() for k, c in pal.items()}, 'class shares', np.bincount(cls.ravel(), minlength=4) / cls.size)
out = np.zeros_like(img)
for k, c in pal.items(): out[cls == k] = c
out = ndi.gaussian_filter(out, sigma=(0.8, 0.8, 0))   # anti-alias panel edges
Image.fromarray((out * 255 + 0.5).astype(np.uint8)).save(dst)
