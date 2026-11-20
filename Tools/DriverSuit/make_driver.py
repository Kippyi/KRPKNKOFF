# Imported driver: racing suit (racing_suit_3d_model.obj, T-pose, helmet fused - removed) + Arai helmet (arai.fbx).
#   blender -b --factory-startup -P Tools/DriverSuit/make_driver.py -- <DriverRig.json> <out dir> [preview prefix]
# DriverRig.json: rest-pose joints dumped by the game (-KartDumpDriverRig), kart frame (m): x forward of the rear axle,
# y left, z up from the ground. The suit is scaled to a 1.80 m driver, cut at the neck, rigged (measured T-pose joints,
# automatic weights), posed onto the game's joints and split into the animated parts:
#   Body (pelvis + legs), Boots, Torso, UpperArmL/R, ForearmL/R  (hands removed: the game keeps its gripping gloves)
# exported in the kart frame. The helmet is exported centred on its bounding box, facing +X, for the head pivot.
import bpy, bmesh, sys, os, json, math
import numpy as np
from mathutils import Vector, Matrix, Quaternion

args = sys.argv[sys.argv.index('--') + 1:]
RIG = json.load(open(args[0])); OUT = args[1]; PREV = args[2] if len(args) > 2 else None
ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '..')
os.makedirs(OUT, exist_ok=True)
T = {k: Vector(v) for k, v in RIG.items() if isinstance(v, list)}
H_TOTAL = 1.80
# tuning (env overrides for A/B previews): ARM_SLIM = radial scale of the baggy AI sleeves round the arm axis (1 = as
# modelled); DQ = dual-quaternion skinning; HELMET_LIFT = helmet raised along the spine above the game's head centre (m)
ARM_SLIM = float(os.environ.get('ARM_SLIM', '0.90')); DQ = os.environ.get('DQ', '0') == '1'
HELMET_LIFT = float(os.environ.get('HELMET_LIFT', '0.05'))

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.wm.obj_import(filepath=os.path.join(ROOT, 'racing_suit_3d_model.obj'))
suit = [o for o in bpy.context.scene.objects if o.type == 'MESH'][0]
suit.name = 'Suit'
me = suit.data
# the OBJ importer puts the Y-up -> Z-up conversion on the object: bake it into the mesh first
me.transform(suit.matrix_world); suit.matrix_world = Matrix.Identity(4); me.update()
V = np.array([v.co[:] for v in me.vertices])
# --- face direction: the boots' toes point forward ---
feet = V[V[:, 2] < 0.04]; ankle_y = V[(V[:, 2] > 0.06) & (V[:, 2] < 0.10)][:, 1].mean()
front_sign = -1.0 if feet[:, 1].mean() < ankle_y else 1.0       # forward = +Y or -Y in the source
# rotate so forward -> +X (left -> +Y), scale to 1.80 m (source height = 1.0)
rot = Matrix.Rotation(math.radians(90.0 if front_sign < 0 else -90.0), 4, 'Z')
me.transform(Matrix.Scale(H_TOTAL / (V[:, 2].max() - V[:, 2].min()), 4) @ rot)
me.update()
V = np.array([v.co[:] for v in me.vertices])
print('suit: %d verts, height %.3f, width (arms) %.3f, depth %.3f' % (len(V), np.ptp(V[:, 2]), np.ptp(V[:, 1]), np.ptp(V[:, 0])))

def slab(z0, z1, cond=None):
    m = (V[:, 2] >= z0) & (V[:, 2] < z1)
    return V[m] if cond is None else V[m & cond]
# --- neck: narrowest horizontal extent between the shoulders and the helmet ---
best = (9, 0)
for z in np.arange(0.80 * H_TOTAL, 0.90 * H_TOTAL, 0.004):
    s = slab(z, z + 0.006, np.abs(V[:, 1]) < 0.15)
    if len(s) < 20: continue
    w = np.ptp(s[:, 1]) + np.ptp(s[:, 0])
    if w < best[0]: best = (w, z)
neck_z = best[1]
# --- arm line (T-pose along y) ---
arm = V[np.abs(V[:, 1]) > 0.30]
arm_z = float(np.median(arm[:, 2])); arm_x = float(np.median(arm[:, 0]))
y_tip = float(np.abs(V[:, 1]).max())
# cuff = narrowest arm cross-section in the outer part (glove cuff)
cuff = (9, 0.0)
for y in np.arange(0.55 * y_tip, 0.85 * y_tip, 0.005):   # beyond ~0.85 the finger roots are narrower than the wrist
    s = V[(np.abs(np.abs(V[:, 1]) - y) < 0.004) & (np.abs(V[:, 2] - arm_z) < 0.12)]
    if len(s) < 8: continue
    r = np.ptp(s[:, 2]) + np.ptp(s[:, 0])
    if r < cuff[0]: cuff = (r, y)
wrist_y = cuff[1]
# torso width BELOW the armpits (a slab at arm height also catches the T-pose arms)
chest = slab(arm_z - 0.30, arm_z - 0.18, np.abs(V[:, 1]) < 0.35)
torso_half = float(np.percentile(np.abs(chest[:, 1]), 97))
shoulder_y = torso_half - 0.01
elbow_y = shoulder_y + 0.54 * (wrist_y - shoulder_y)
# --- legs: crotch = highest z with a gap between the legs ---
crotch = 0.0
for z in np.arange(0.30 * H_TOTAL, 0.60 * H_TOTAL, 0.005):
    s = slab(z, z + 0.006, np.abs(V[:, 1]) < 0.20)
    if len(s) and not np.any(np.abs(s[:, 1]) < 0.012): crotch = z
hip_z = crotch + 0.07
def leg_centre(z, side):
    s = slab(z - 0.01, z + 0.01, (V[:, 1] * side > 0.01) & (np.abs(V[:, 1]) < 0.25))
    return Vector((float(np.mean(s[:, 0])), float(np.mean(s[:, 1])), z))
knee_z = 0.285 * H_TOTAL; ankle_z = 0.050 * H_TOTAL
toe_x = float(V[V[:, 2] < 0.05][:, 0].max())
torso_x = float(np.mean(slab(hip_z, neck_z, np.abs(V[:, 1]) < 0.12)[:, 0]))
print('T-pose joints: neck %.3f, arm z %.3f, shoulder %.3f, elbow %.3f, wrist %.3f, tip %.3f, crotch %.3f, hip %.3f' % (
    neck_z, arm_z, shoulder_y, elbow_y, wrist_y, y_tip, crotch, hip_z))

# --- remove the fused helmet: everything above a slanted neck plane ---
# The helmet's lower edge is not level: its chin bar starts ~3 cm below the narrowest neck slice at the front and its
# rear rim ends ~2 cm above it at the back (measured radius / texture-colour profile round the neck axis). A level cut
# left a chin flap in front and a rim flap behind ("collar peak"), so the plane drops 0.25 m per m towards the front.
neck_ring = slab(neck_z - 0.01, neck_z + 0.01, np.abs(V[:, 1]) < 0.15)
neck_cx = float(np.median(neck_ring[:, 0]))
def neck_cut_z(x): return neck_z - 0.010 - 0.25 * (x - neck_cx)
bm = bmesh.new(); bm.from_mesh(me)
bmesh.ops.bisect_plane(bm, geom=bm.verts[:] + bm.edges[:] + bm.faces[:], plane_co=Vector((neck_cx, 0.0, neck_cut_z(neck_cx))),
                       plane_no=Vector((0.25, 0.0, 1.0)).normalized(), clear_outer=True)   # clean edge (no vertex zigzag)
bm.to_mesh(me); bm.free(); me.update()

# --- slim the sleeves: the AI suit's arms are puffy (they read as chunky shoulders once posed forward). Scale radially
# about the T-pose arm axis (y), ramping in from inside the shoulder cap to full past it, so the cap slopes too.
if ARM_SLIM != 1.0:
    for v in me.vertices:
        ay = abs(v.co.y)
        if v.co.z < hip_z or ay < shoulder_y - 0.04: continue
        if math.hypot(v.co.x - arm_x, v.co.z - arm_z) > 0.13: continue   # arm tube only (not the torso flank)
        t = min(1.0, (ay - (shoulder_y - 0.04)) / 0.12); k = 1.0 - (1.0 - ARM_SLIM) * t
        v.co.x = arm_x + (v.co.x - arm_x) * k; v.co.z = arm_z + (v.co.z - arm_z) * k
    me.update(); V = np.array([v.co[:] for v in me.vertices])
    print('sleeves slimmed to %.2f' % ARM_SLIM)
# --- shoulder slope: the AI suit's shoulder tops run flat and level from the collar out to the arms (boxy "pads" in
# front view). Real shoulders fall from the neck to the deltoid (trapezius slope, visual target: 0.035 read boxy, 0.055 with 0.82 sleeves read too slim). Lower
# the top surface progressively outward from the neck, fading out along the upper arm.
SHOULDER_DROP = float(os.environ.get('SHOULDER_DROP', '0.045'))
def _ss(a, b, x):
    t = min(1.0, max(0.0, (x - a) / (b - a))); return t * t * (3 - 2 * t)
if SHOULDER_DROP > 0:
    for v in me.vertices:
        ay = abs(v.co.y)
        if v.co.z < arm_z - 0.06 or v.co.z > neck_z + 0.05: continue
        d = SHOULDER_DROP * _ss(0.07, 0.20, ay) * (1.0 - _ss(0.24, 0.36, ay)) * _ss(arm_z - 0.05, arm_z + 0.05, v.co.z)
        v.co.z -= d
    me.update(); V = np.array([v.co[:] for v in me.vertices])
    print('shoulder slope %.3f m' % SHOULDER_DROP)

# --- armature at the measured T-pose joints ---
J = {}
J['pelvis'] = Vector((torso_x, 0, hip_z)); J['spine'] = Vector((torso_x, 0, hip_z + 0.10))
J['chest'] = Vector((torso_x, 0, arm_z - 0.12)); J['neck'] = Vector((torso_x, 0, neck_z))
for sd, sg in (('L', 1), ('R', -1)):
    J['shoulder' + sd] = Vector((arm_x, sg * shoulder_y, arm_z)); J['elbow' + sd] = Vector((arm_x, sg * elbow_y, arm_z))
    J['wrist' + sd] = Vector((arm_x, sg * wrist_y, arm_z)); J['tip' + sd] = Vector((arm_x, sg * y_tip, arm_z))
    J['hip' + sd] = leg_centre(hip_z - 0.12, sg); J['hip' + sd].z = hip_z
    J['knee' + sd] = leg_centre(knee_z, sg); J['ankle' + sd] = leg_centre(ankle_z, sg)
    J['toe' + sd] = Vector((toe_x - 0.03, J['ankle' + sd].y, 0.03))
BONES = [('pelvis', 'pelvis', 'spine', None), ('spine', 'spine', 'chest', 'pelvis'), ('chest', 'chest', 'neck', 'spine')]
for sd in 'LR':
    BONES += [('upperarm' + sd, 'shoulder' + sd, 'elbow' + sd, 'chest'), ('forearm' + sd, 'elbow' + sd, 'wrist' + sd, 'upperarm' + sd),
              ('hand' + sd, 'wrist' + sd, 'tip' + sd, 'forearm' + sd), ('thigh' + sd, 'hip' + sd, 'knee' + sd, 'pelvis'),
              ('shin' + sd, 'knee' + sd, 'ankle' + sd, 'thigh' + sd), ('foot' + sd, 'ankle' + sd, 'toe' + sd, 'shin' + sd)]
arm_data = bpy.data.armatures.new('Rig'); arm_obj = bpy.data.objects.new('Rig', arm_data)
bpy.context.scene.collection.objects.link(arm_obj)
bpy.context.view_layer.objects.active = arm_obj; bpy.ops.object.mode_set(mode='EDIT')
for name, h, t, parent in BONES:
    b = arm_data.edit_bones.new(name); b.head = J[h]; b.tail = J[t]
    if parent: b.parent = arm_data.edit_bones[parent]; b.use_connect = False
bpy.ops.object.mode_set(mode='OBJECT')
bpy.ops.object.select_all(action='DESELECT'); suit.select_set(True); arm_obj.select_set(True)
bpy.context.view_layer.objects.active = arm_obj
bpy.ops.object.parent_set(type='ARMATURE_NAME')   # weights assigned below
for md in suit.modifiers:
    if md.type == 'ARMATURE': md.use_deform_preserve_volume = DQ   # DQ bulged the shoulder caps upward; linear blend by default
groups = {g.index: g.name for g in suit.vertex_groups}
weighted = sum(1 for v in me.vertices if len(v.groups))
print('automatic weights: %d / %d vertices weighted' % (weighted, len(me.vertices)))
if True:   # bone heat fails on this AI-generated mesh (0 weighted): masked distance weights
    # bone heat failed on part of the mesh: envelope-style fallback (distance to bone segments)
    for vg in list(suit.vertex_groups): suit.vertex_groups.remove(vg)
    segs = [(n, J[h], J[t]) for n, h, t, _ in BONES]
    vgs = {n: suit.vertex_groups.new(name=n) for n, _, _ in segs}
    for v in me.vertices:
        d = []
        ay = abs(v.co.y)
        for n, a, b in segs:
            # anatomical masks: nothing inside the torso width belongs to an arm; nothing above the hips to a leg;
            # the arms (outside the torso) never belong to the torso / pelvis
            if n.startswith(('upperarm', 'forearm', 'hand')) and ay < shoulder_y - 0.03: continue
            if n.startswith(('thigh', 'shin', 'foot')) and v.co.z > hip_z + 0.06: continue
            if n in ('pelvis', 'spine', 'chest') and ay > shoulder_y + 0.04 and v.co.z > arm_z - 0.15: continue
            if n.startswith(('upperarm', 'forearm', 'hand')) and n[-1] == ('L' if v.co.y < 0 else 'R'): continue
            if n.startswith(('thigh', 'shin', 'foot')) and n[-1] == ('L' if v.co.y < -0.02 else ('R' if v.co.y > 0.02 else '?')): continue
            ab = b - a; tt = max(0.0, min(1.0, (v.co - a).dot(ab) / ab.length_squared)); d.append((n, (v.co - (a + ab * tt)).length))
        if len(d) < 2: d.append(d[0])
        d.sort(key=lambda x: x[1]); w0 = 1.0 / (d[0][1] + 1e-3) ** 4; w1 = 1.0 / (d[1][1] + 1e-3) ** 4
        vgs[d[0][0]].add([v.index], w0 / (w0 + w1), 'REPLACE'); vgs[d[1][0]].add([v.index], w1 / (w0 + w1), 'REPLACE')
    # The masks above give a hard arm/torso seam; posing the T-pose arms forward then stretched a band of faces across
    # the shoulder blades (texture smeared into streaks). Diffuse the weights over the mesh surface (edge graph, so
    # nothing leaks between surfaces that only touch in space) across ~6 cm, then keep the 4 strongest influences.
    names = [n for n, _, _ in segs]; gi = {suit.vertex_groups[n].index: k for k, n in enumerate(names)}
    Wm = np.zeros((len(me.vertices), len(names)))
    for v in me.vertices:
        for g in v.groups: Wm[v.index, gi[g.group]] = g.weight
    E = np.array([e.vertices[:] for e in me.edges])
    elen = float(np.mean([(me.vertices[a].co - me.vertices[b].co).length for a, b in E[::50]]))
    iters = int(min(400, max(20, (0.06 / elen) ** 2)))
    deg = np.bincount(E.ravel(), minlength=len(Wm)).astype(float)[:, None]; deg[deg == 0] = 1
    for _ in range(iters):
        nb = np.zeros_like(Wm)
        for k in range(Wm.shape[1]):
            nb[:, k] = np.bincount(E[:, 0], Wm[E[:, 1], k], len(Wm)) + np.bincount(E[:, 1], Wm[E[:, 0], k], len(Wm))
        Wm = 0.5 * Wm + 0.5 * nb / deg
    Wm[Wm < np.sort(Wm, axis=1)[:, [-4]]] = 0.0
    Wm /= np.maximum(Wm.sum(1, keepdims=True), 1e-9)
    for vg in list(suit.vertex_groups): suit.vertex_groups.remove(vg)
    vgs = {n: suit.vertex_groups.new(name=n) for n in names}
    for k, n in enumerate(names):
        for w in np.unique(np.round(Wm[:, k], 3)):
            if w <= 0: continue
            vgs[n].add(np.nonzero(np.round(Wm[:, k], 3) == w)[0].tolist(), float(w), 'REPLACE')
    print('weights smoothed: mean edge %.4f m, %d iterations' % (elen, iters))
    groups = {g.index: g.name for g in suit.vertex_groups}
    print('fallback distance weights used')

# --- pose onto the game's rest joints (kart frame) ---
tgt = {'pelvis': (T['pelvis'], T['neck']), 'spine': (None, T['neck']), 'chest': (None, T['neck'])}
for sd in 'LR':
    tgt['upperarm' + sd] = (T['shoulder' + sd], T['elbow' + sd]); tgt['forearm' + sd] = (None, T['wrist' + sd])
    tgt['hand' + sd] = (None, T['grip' + sd]); tgt['thigh' + sd] = (T['hip' + sd], T['knee' + sd])
    tgt['shin' + sd] = (None, T['ankle' + sd]); tgt['foot' + sd] = (None, None)
bpy.context.view_layer.objects.active = arm_obj; bpy.ops.object.mode_set(mode='POSE')
pb = arm_obj.pose.bones
stretch = {'upperarm', 'forearm', 'thigh', 'shin'}
def set_bone(name, head_target, tail_target, direction=None):
    p = pb[name]; bpy.context.view_layer.update()
    rest = p.bone.matrix_local.copy()
    head = head_target if head_target is not None else p.head.copy()
    if direction is None: direction = (tail_target - head)
    q = rest.to_quaternion() @ Vector((0, 1, 0))
    rot = q.rotation_difference(direction.normalized()) @ rest.to_quaternion()
    s = 1.0
    if any(name.startswith(k) for k in stretch) and tail_target is not None: s = direction.length / p.bone.length
    p.matrix = Matrix.Translation(head) @ rot.to_matrix().to_4x4() @ Matrix.Diagonal((1.0, s, 1.0, 1.0))
    bpy.context.view_layer.update()
spine_dir = T['neck'] - T['pelvis']
set_bone('pelvis', T['pelvis'], None, spine_dir)
# The suit's shoulder line sits higher above its (estimated) hip than the game's shoulders above the seat pelvis, which
# pushed the collar ~12 cm up into the helmet. Drop the spine/chest along the spine so the suit's shoulder line lands
# on the game's shoulder joints (the pelvis stays on the seat; the waist absorbs the difference).
spine_u = spine_dir.normalized()
game_sh = ((T['shoulderL'] + T['shoulderR']) * 0.5 - T['pelvis']).dot(spine_u)
torso_drop = max(0.0, (arm_z - hip_z) - game_sh)
bpy.context.view_layer.update()
set_bone('spine', pb['spine'].head - spine_u * torso_drop, None, spine_dir); set_bone('chest', None, None, spine_dir)
print('torso drop %.3f m (suit hip->shoulder %.3f, game %.3f)' % (torso_drop, arm_z - hip_z, game_sh))
for sd in 'LR':
    set_bone('thigh' + sd, T['hip' + sd], T['knee' + sd])
    set_bone('shin' + sd, None, T['ankle' + sd])
    set_bone('foot' + sd, None, None, Vector((0.64, 0.0, 0.77)))            # sole up against the pedal face
    set_bone('upperarm' + sd, T['shoulder' + sd], T['elbow' + sd])
    set_bone('forearm' + sd, None, T['wrist' + sd])
    set_bone('hand' + sd, None, T['grip' + sd])
bpy.ops.object.mode_set(mode='OBJECT')
for sd in 'LR':
    print('posed %s: shoulder err %.3f m, elbow err %.3f m, wrist err %.3f m, knee err %.3f, ankle err %.3f' % (sd,
        (pb['upperarm' + sd].head - T['shoulder' + sd]).length, (pb['forearm' + sd].head - T['elbow' + sd]).length,
        (pb['forearm' + sd].tail - T['wrist' + sd]).length, (pb['shin' + sd].head - T['knee' + sd]).length,
        (pb['shin' + sd].tail - T['ankle' + sd]).length))

# --- bake the pose, split by dominant bone ---
dg = bpy.context.evaluated_depsgraph_get()
posed = bpy.data.meshes.new_from_object(suit.evaluated_get(dg))
dom = []
for v in me.vertices:
    best_g, best_w = None, -1
    for g in v.groups:
        if g.weight > best_w: best_g, best_w = groups.get(g.group), g.weight
    # hands are cut geometrically at the measured T-pose wrist (smoothed weights alone leave finger fragments)
    if abs(v.co.y) > wrist_y and v.co.z > hip_z: best_g = 'hand' + ('L' if v.co.y > 0 else 'R')
    dom.append(best_g or 'pelvis')
PART = {'pelvis': 'Body', 'thighL': 'Body', 'thighR': 'Body', 'shinL': 'Body', 'shinR': 'Body', 'footL': 'Boots', 'footR': 'Boots',
        'spine': 'Torso', 'chest': 'Torso', 'upperarmL': 'UpperArmL', 'upperarmR': 'UpperArmR',
        'forearmL': 'ForearmL', 'forearmR': 'ForearmR', 'handL': None, 'handR': None}
CLASS = {'Body': 'suittex', 'Boots': 'boottex', 'Torso': 'suittex', 'UpperArmL': 'suittex', 'UpperArmR': 'suittex', 'ForearmL': 'suittex', 'ForearmR': 'suittex'}   # textured (human) / flat fallback (AI)
written = []
for part in sorted(set(p for p in PART.values() if p)):
    bm = bmesh.new(); bm.from_mesh(posed)
    bm.verts.ensure_lookup_table()
    keep = []
    for f in bm.faces:
        votes = {}
        for v in f.verts: votes[PART[dom[v.index]]] = votes.get(PART[dom[v.index]], 0) + 1
        if max(votes, key=votes.get) == part: keep.append(f)
    bmesh.ops.delete(bm, geom=[f for f in bm.faces if f not in set(keep)], context='FACES')
    pm = bpy.data.meshes.new('Driver_' + part); bm.to_mesh(pm); bm.free()
    pm.materials.append(bpy.data.materials.new('MI_' + CLASS[part]))
    ob = bpy.data.objects.new('Driver_' + part, pm); bpy.context.scene.collection.objects.link(ob)
    bpy.ops.object.select_all(action='DESELECT'); ob.select_set(True); bpy.context.view_layer.objects.active = ob
    bpy.ops.export_scene.fbx(filepath=os.path.join(OUT, 'Driver_' + part + '.fbx'), use_selection=True, apply_unit_scale=True,
                             apply_scale_options='FBX_SCALE_ALL', mesh_smooth_type='FACE', add_leaf_bones=False, bake_anim=False,
                             axis_forward='-Z', axis_up='Y')
    written.append({'part': part, 'class': CLASS[part], 'asset': 'Driver_' + part, 'tris': len(pm.polygons)})
    print('PART', written[-1])

# --- neck: the suit's own neck was inside the fused helmet. A balaclava-covered neck (tapered tube, matte black) rises
# from inside the posed collar ring into the helmet; it rides on the torso. Radius from a 1.80 m adult neck
# (circumference ~0.38 m -> r ~0.06 m, ESTIMATE), capped to fit inside the collar.
ring_idx = [v.index for v in me.vertices if v.co.z > neck_cut_z(v.co.x) - 0.012 and abs(v.co.y) < 0.14]
ring = np.array([posed.vertices[i].co[:] for i in ring_idx])
collar_c = Vector(ring.mean(0).tolist())
helm_c = T['helmetCentre']
axis = (helm_c - collar_c).normalized()
rel = ring - np.array(collar_c[:])
radial = rel - np.outer(rel @ np.array(axis[:]), np.array(axis[:]))
collar_r = float(np.median(np.linalg.norm(radial, axis=1)))
collar_top = float((rel @ np.array(axis[:])).max())
neck_r = min(0.060, 0.88 * collar_r)
n0 = collar_c - axis * 0.05; n1 = helm_c + spine_u * HELMET_LIFT - axis * 0.03
bm = bmesh.new()
bmesh.ops.create_cone(bm, cap_ends=False, segments=24, radius1=neck_r, radius2=neck_r * 0.92, depth=(n1 - n0).length)
rotq = Vector((0, 0, 1)).rotation_difference(axis)
bmesh.ops.transform(bm, matrix=Matrix.Translation((n0 + n1) * 0.5) @ rotq.to_matrix().to_4x4(), verts=bm.verts)
pm = bpy.data.meshes.new('Driver_Neck'); bm.to_mesh(pm); bm.free()
for p in pm.polygons: p.use_smooth = True
pm.materials.append(bpy.data.materials.new('MI_rubber'))
ob = bpy.data.objects.new('Driver_Neck', pm); bpy.context.scene.collection.objects.link(ob)
bpy.ops.object.select_all(action='DESELECT'); ob.select_set(True); bpy.context.view_layer.objects.active = ob
bpy.ops.export_scene.fbx(filepath=os.path.join(OUT, 'Driver_Neck.fbx'), use_selection=True, apply_unit_scale=True,
                         apply_scale_options='FBX_SCALE_ALL', mesh_smooth_type='FACE', add_leaf_bones=False, bake_anim=False,
                         axis_forward='-Z', axis_up='Y')
written.append({'part': 'Neck', 'class': 'rubber', 'asset': 'Driver_Neck', 'tris': len(pm.polygons)})
print('neck: collar centre %s r %.3f, collar top %.3f along axis, helmet centre %.3f above collar, neck r %.3f' % (
    tuple(round(c, 3) for c in collar_c), collar_r, collar_top, (helm_c - collar_c).length, neck_r))

# --- helmet: Arai GP7 (arai.fbx), centred, facing +X ---
before = set(bpy.context.scene.objects)
bpy.ops.import_scene.fbx(filepath=os.path.join(ROOT, 'arai.fbx'))
helm = [o for o in bpy.context.scene.objects if o not in before and o.type == 'MESH']
pts = np.array([list(o.matrix_world @ Vector(c)) for o in helm for v in o.data.vertices for c in [v.co]])
centre = Vector(((pts.min(0) + pts.max(0)) / 2).tolist())
# forward: the visor (material with 'visor.tga') sits on the front
vis = []
for o in helm:
    mw = o.matrix_world
    for poly in o.data.polygons:
        m = o.material_slots[poly.material_index].material if poly.material_index < len(o.material_slots) else None
        if m and m.use_nodes and any(n.type == 'TEX_IMAGE' and n.image and 'visor.tga' in n.image.filepath.lower() for n in m.node_tree.nodes):
            vis.append(mw @ poly.center)
vc = (sum(vis, Vector()) / len(vis)) - centre if vis else Vector((0, 1, 0))
yaw = math.atan2(vc.y, vc.x)
fix = Matrix.Rotation(-yaw, 4, 'Z') @ Matrix.Translation(-centre)
for o in helm:
    o.data.transform(fix @ o.matrix_world); o.matrix_world = Matrix.Identity(4)
bpy.ops.object.select_all(action='DESELECT')
for o in helm: o.select_set(True)
bpy.context.view_layer.objects.active = helm[0]
bpy.ops.export_scene.fbx(filepath=os.path.join(OUT, 'Helmet_Arai.fbx'), use_selection=True, apply_unit_scale=True,
                         apply_scale_options='FBX_SCALE_ALL', mesh_smooth_type='FACE', add_leaf_bones=False, bake_anim=False,
                         axis_forward='-Z', axis_up='Y', path_mode='ABSOLUTE')
size = pts.max(0) - pts.min(0)
print('helmet: %d objects, size %.3f x %.3f x %.3f m, visor yaw %.0f deg' % (len(helm), *size, math.degrees(yaw)))
json.dump({'parts': written, 'helmet': {'asset': 'Helmet_Arai'}, 'neckCut': neck_z, 'helmetLift': HELMET_LIFT}, open(os.path.join(OUT, 'driver_layout.json'), 'w'), indent=1)

# --- preview: posed parts in the kart frame (workbench), side + front-left ---
if PREV:
    for o in list(bpy.context.scene.objects):
        if o.type == 'MESH' and not o.name.startswith('Driver_') and o not in helm: o.hide_render = True
    for o in helm: o.matrix_world = Matrix.Translation(T['helmetCentre'])
    arm_obj.hide_render = True
    sc = bpy.context.scene; sc.render.engine = 'BLENDER_WORKBENCH'; sc.render.resolution_x = 900; sc.render.resolution_y = 700
    sc.display.shading.light = 'STUDIO'; sc.display.shading.color_type = 'RANDOM'
    cam = bpy.data.objects.new('cam', bpy.data.cameras.new('cam')); sc.collection.objects.link(cam); sc.camera = cam
    cam.data.type = 'ORTHO'; cam.data.ortho_scale = 1.6; c = Vector((0.65, 0, 0.42))
    for nm, d in (('side', Vector((0, -1, 0))), ('frontleft', Vector((1, 0.8, 0.5)).normalized())):
        cam.location = c + d * 4; cam.rotation_euler = (c - cam.location).to_track_quat('-Z', 'Y').to_euler()
        sc.render.filepath = PREV + '_' + nm + '.png'; bpy.ops.render.render(write_still=True)
