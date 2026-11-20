# Splits "kips bog bog kart.fbx" (Blender kitbash, 61 objects) into game parts for the kart visual rig.
# Run headless:
#   blender -b --factory-startup -P Tools/SplitBogKart.py -- "<fbx>" "<out dir>"
#
# Output (one FBX per part, a single material slot named after its material class):
#   Body_<class>.fbx          static parts, pivot at the model's rear-axle centre on the kart centreline
#   Wheel<FL|FR|RL|RR>_<class> pivot at the hub centre
#   Steer_<class>.fbx         steering wheel + display, pivot on the column axis at the hub
#   BogKartLayout.json        measured pivots / radii / column (metres, kart frame: x fwd, y left, z up)
# All geometry is re-expressed in "kart frame" before export: forward = +X, left = +Y, up = +Z
# (the model is authored forward = +Y, right = +X). The source textures are not available
# (paths point to another PC), so materials are reduced to classes and coloured at runtime.
import bpy, bmesh, sys, os, json, mathutils

argv = sys.argv[sys.argv.index('--') + 1:]
SRC, OUT = argv[0], argv[1]
os.makedirs(OUT, exist_ok=True)
bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=SRC)
objs = {o.name: o for o in bpy.context.scene.objects if o.type == 'MESH' and len(o.data.polygons) > 0}

WHEELS = {
    'FL': ['Cylinder.004', 'Cylinder.027', 'Cylinder.025'],
    'FR': ['Cylinder.008', 'Cylinder.006', 'Cylinder.007'],
    'RL': ['Cylinder.010', 'Cylinder.011'],
    'RR': ['Cylinder.009', 'Cylinder.012'],
}
TYRES = {'Cylinder.004', 'Cylinder.008', 'Cylinder.009', 'Cylinder.010'}
STEER = ['SWHEEL', 'Plane.017']
# Moving suspension / driveline parts (animated in game): spindles steer with the front wheels (no spin), the rear
# hub sleeves + brake disc spin with the axle, tie rods run from the steering-column arm to each spindle arm.
SPINDLES = {'L': ['SPINDLE_L'], 'R': ['SPINDLE_R']}
REAR_SPIN = ['frame.011', 'REARDISC.001']
TIE_RODS = {'L': 'bog', 'R': 'bog2'}       # bog: model x < 0 = kart left
KINGPIN_BRACKETS = 'frame.014'              # anodised kingpin bosses on the chassis: their centres are the kingpins
# Default class per object (overridden per material by keyword below).
BODYWORK = {'fbumper', 'SIDEPOD_L', 'SIDEPOD_R', 'rbumper', 'Plane', 'nasau no UV', 'nassau intake'}
PLASTIC = {'Plane.001', 'Cube', 'NIGGER', 'kg', 'frame'}   # front panel bits, seat (renamed), tank, floor tray
CHASSIS = {'frame.001'}
DISPLAY = {'Plane.017'}

def base(n):
    n = n.split('.')[0] if n[-4:-3] == '.' and n[-3:].isdigit() else n
    return n.lower()

# R2S livery textures supplied by the user (project root *.tga): parts whose source material used one of them keep
# their UVs and export as a "skin*" class (textured clear-coat paint in game). Objects without UVs keep paint.
SKIN = {'r2s_kg509_front': 'skinfront', 'r2s_kg507_sidepod': 'skinside', 'r2s_kg_rear': 'skinrear',
        'sodi swheel': 'swgrip', 'm7 base uv': 'swbase'}

def skin_class(obj, slot):
    mat = obj.material_slots[slot].material if slot < len(obj.material_slots) else None
    if not mat or not mat.use_nodes or not obj.data.uv_layers:
        return None
    for n in mat.node_tree.nodes:
        if n.type == 'TEX_IMAGE' and n.image and any(l.to_socket.name == 'Base Color' for l in n.outputs[0].links):
            key = os.path.splitext(os.path.basename(n.image.filepath))[0].lower().split('.')[0]
            if key in SKIN:
                return SKIN[key]
    return None

def mat_class(obj_name, mat_name):
    m = base(mat_name)
    if obj_name in TYRES: return 'rubber'
    if obj_name in DISPLAY: return 'display'
    if 'chrome' in m or 'steel' in m or 'thread' in m: return 'chrome'
    if 'alloy' in m: return 'alloy'
    if 'annodized' in m or 'anodized' in m: return 'anodized'
    if obj_name in BODYWORK:
        return 'chrome' if 'metal_bar' in m else 'body'
    if obj_name in CHASSIS or 'chas' in m: return 'chassis'
    if obj_name in PLASTIC: return 'plastic'
    if 'metal_bar' in m: return 'chrome'
    if obj_name in sum(WHEELS.values(), []): return 'alloy'
    return 'graphite'

def world_bounds(names):
    pts = [objs[n].matrix_world @ mathutils.Vector(c) for n in names for c in objs[n].bound_box]
    return [min(p[i] for p in pts) for i in range(3)], [max(p[i] for p in pts) for i in range(3)]

# --- Measurements (model frame: x right, y fwd, z up, metres) ---
hub, radius = {}, {}
for k, names in WHEELS.items():
    tyre = [n for n in names if n in TYRES]
    mn, mx = world_bounds(tyre)
    hub[k] = mathutils.Vector([(a + b) / 2 for a, b in zip(mn, mx)])
    radius[k] = ((mx[1] - mn[1]) + (mx[2] - mn[2])) / 4.0
rear_axle = (hub['RL'] + hub['RR']) / 2
rear_axle.x = 0.0
# Steering column axis: longest local axis of the column object.
col = bpy.context.scene.objects['SCOLUMN']
ext = [max(c[i] for c in col.bound_box) - min(c[i] for c in col.bound_box) for i in range(3)]
ai = ext.index(max(ext))
axis = (col.matrix_world.to_3x3() @ mathutils.Vector([1.0 if i == ai else 0.0 for i in range(3)])).normalized()
if axis.z < 0: axis = -axis                                    # pointing up toward the driver
c0 = col.matrix_world @ (sum((mathutils.Vector(c) for c in col.bound_box), mathutils.Vector()) / 8)
mn, mx = world_bounds(['SWHEEL'])
sw_c = mathutils.Vector([(a + b) / 2 for a, b in zip(mn, mx)])
hub_on_axis = c0 + axis * (sw_c - c0).dot(axis)               # steering hub: column-axis point nearest the wheel centre
col_base = c0 - axis * (ext[ai] * col.matrix_world.to_scale()[ai] * 0.5)

def to_kart(p):   # model point -> kart frame (x fwd of rear axle, y left, z up; z kept relative to the axle centre)
    d = p - rear_axle
    return [d.y, -d.x, d.z]
def to_kart_dir(v):
    return [v.y, -v.x, v.z]

layout = {
    'note': 'Measured from the FBX by Tools/SplitBogKart.py. Kart frame relative to the rear-axle centre (z = 0 at axle height).',
    'hubs': {k: to_kart(v) for k, v in hub.items()},
    'tyreRadius': radius,
    'wheelbase': hub['FL'].y - hub['RL'].y,
    'frontTrack': hub['FR'].x - hub['FL'].x,
    'rearTrack': hub['RR'].x - hub['RL'].x,
    'steerHub': to_kart(hub_on_axis),
    'columnBase': to_kart(col_base),
    'columnAxis': to_kart_dir(axis),
}

# Kart-frame basis change: new = M @ (p - pivot); fwd(+Y)->+X, right(+X)->-Y (left = +Y), up->up.
M = mathutils.Matrix(((0, 1, 0, 0), (-1, 0, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)))

def export(part_name, names, pivot):
    by_class = {}
    for n in names:
        o = objs[n]
        # One UV layer named UVMap (joined meshes must share the layer name).
        if o.data.uv_layers:
            act = o.data.uv_layers.active
            for l in [l for l in o.data.uv_layers if l != act]:
                o.data.uv_layers.remove(l)
            o.data.uv_layers[0].name = 'UVMap'
        for si, s in enumerate(o.material_slots):
            cls = skin_class(o, si) or mat_class(n, s.material.name if s.material else '')
            by_class.setdefault(cls, []).append((o, si))
        if not o.material_slots:
            by_class.setdefault(mat_class(n, ''), []).append((o, -1))
    written = []
    for cls, items in by_class.items():
        bm_all = bmesh.new()
        for o, si in items:
            bm = bmesh.new()
            bm.from_mesh(o.data)
            if si >= 0:
                bmesh.ops.delete(bm, geom=[f for f in bm.faces if f.material_index != si], context='FACES')
            bm.transform(mathutils.Matrix.Translation(-pivot) @ o.matrix_world)
            if o.matrix_world.determinant() < 0:
                bmesh.ops.reverse_faces(bm, faces=bm.faces)
            tmp = bpy.data.meshes.new('tmp')
            bm.to_mesh(tmp); bm.free()
            bm_all.from_mesh(tmp)
            bpy.data.meshes.remove(tmp)
        bm_all.transform(M)                      # det(M) = +1: a proper rotation, no face flip needed
        for f in bm_all.faces: f.material_index = 0
        name = f'{part_name}_{cls}'
        me = bpy.data.meshes.new(name)
        bm_all.to_mesh(me); bm_all.free()
        mat = bpy.data.materials.get('MI_' + cls) or bpy.data.materials.new('MI_' + cls)
        me.materials.append(mat)
        ob = bpy.data.objects.new(name, me)
        bpy.context.scene.collection.objects.link(ob)
        bpy.ops.object.select_all(action='DESELECT')
        ob.select_set(True)
        bpy.context.view_layer.objects.active = ob
        # FBX default axes (-Z forward, Y up) + UE's importer conversion map Blender (x,y,z) to UE (x,-y,z):
        # with forward=+X, left=+Y in Blender this lands as UE forward=+X, right=+Y, up=+Z.
        bpy.ops.export_scene.fbx(filepath=os.path.join(OUT, name + '.fbx'), use_selection=True,
                                 apply_unit_scale=True, apply_scale_options='FBX_SCALE_ALL',
                                 mesh_smooth_type='FACE', add_leaf_bones=False, bake_anim=False,
                                 axis_forward='-Z', axis_up='Y')
        bpy.data.objects.remove(ob)
        written.append({'part': part_name, 'class': cls, 'asset': name, 'tris': len(me.polygons)})
    return written

wheel_objs = sum(WHEELS.values(), [])
moving = sum(SPINDLES.values(), []) + REAR_SPIN + list(TIE_RODS.values())
body = [n for n in objs if n not in wheel_objs and n not in STEER and n not in moving]

# Tie-rod end points from the real vertices (outer = spindle arm end, inner = steering-column arm end).
import numpy as np
tie = {}
for side, name in TIE_RODS.items():
    o = objs[name]
    v = np.array([list(o.matrix_world @ x.co) for x in o.data.vertices])
    ax = np.abs(v[:, 0])
    outer = v[ax > ax.max() - 0.012].mean(0); inner = v[ax < ax.min() + 0.012].mean(0)
    tie[side] = (mathutils.Vector(outer), mathutils.Vector(inner))
layout['tieRods'] = {s_: {'outer': to_kart(o_), 'inner': to_kart(i_)} for s_, (o_, i_) in tie.items()}
kb = objs[KINGPIN_BRACKETS]
kv = np.array([list(kb.matrix_world @ x.co) for x in kb.data.vertices])
kingpin = {}
for side, sel in (('L', kv[:, 0] < 0), ('R', kv[:, 0] > 0)):
    a = kv[sel]; kingpin[side] = mathutils.Vector((a.min(0) + a.max(0)) / 2)
layout['kingpins'] = {s_: to_kart(v_) for s_, v_ in kingpin.items()}
parts = []
parts += export('Body', body, rear_axle)
for k, names in WHEELS.items():
    parts += export('Wheel' + k, names, hub[k])
parts += export('Steer', STEER, hub_on_axis)
parts += export('SpindleL', SPINDLES['L'], kingpin['L'])   # pivot = kingpin: the spindle stays on the chassis,
parts += export('SpindleR', SPINDLES['R'], kingpin['R'])   # the hub / wheel slides along the stub axle with the track setting
parts += export('RearSpin', REAR_SPIN, rear_axle)
for side, name in TIE_RODS.items():
    parts += export('TieRod' + side, [name], tie[side][0])
layout['parts'] = parts
json.dump(layout, open(os.path.join(OUT, 'BogKartLayout.json'), 'w'), indent=2)
print('LAYOUT', json.dumps({k: v for k, v in layout.items() if k != 'parts'}, indent=1))
for p in parts: print('PART', p)
