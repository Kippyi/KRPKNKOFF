# Downloads CC0 models from Poly Haven (glTF, 2k textures) into Art/Models/PolyHaven/<id>/.
#   python -I Tools/FetchPolyHaven.py <dest dir> id [id ...]
# Poly Haven assets are CC0 (public domain): https://polyhaven.com/license
import json, os, sys, urllib.request

UA = {'User-Agent': 'kart-sim-asset-fetch'}
def get(url):
    with urllib.request.urlopen(urllib.request.Request(url, headers=UA), timeout=120) as r:
        return r.read()

dest = sys.argv[1]
for aid in sys.argv[2:]:
    files = json.loads(get(f'https://api.polyhaven.com/files/{aid}'))
    entry = files['gltf']['2k']['gltf']
    out = os.path.join(dest, aid)
    os.makedirs(out, exist_ok=True)
    items = [(os.path.basename(entry['url']), entry['url'])] + [(rel, inc['url']) for rel, inc in entry.get('include', {}).items()]
    total = 0
    for rel, url in items:
        norm = os.path.normpath(rel)
        if norm.startswith('..') or os.path.isabs(norm):
            raise SystemExit(f'unsafe path in {aid}: {rel}')
        path = os.path.join(out, norm)
        os.makedirs(os.path.dirname(path), exist_ok=True)
        if not os.path.exists(path):
            data = get(url)
            with open(path, 'wb') as f:
                f.write(data)
        total += os.path.getsize(path)
    print(f'{aid}: {len(items)} files, {total / 1e6:.1f} MB')
with open(os.path.join(dest, 'LICENSE.txt'), 'w') as f:
    f.write('Models from Poly Haven (polyhaven.com), licensed CC0 1.0 Universal (public domain). No attribution required.\n')
