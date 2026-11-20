# Splits the front fairing (nose) out of Art/BogKart/Body_skinfront.fbx so the game can push it back when the CIK
# front fairing kit is triggered (Milestone 8 damage). The skinfront mesh holds the nose (largest loose part, ahead of
# x = 1.25 m, low) and the front panel (up to the steering column): the nose becomes Nose_skinfront.fbx and the rest is
# re-exported as Body_skinfront.fbx. Adds the Nose part to Art/BogKart/BogKartLayout.json.
#   "C:/Program Files/Blender Foundation/Blender 4.3/blender.exe" -b -P Tools/SplitNose.py
# Then: UnrealEditor-Cmd ... -script="Tools/ImportNose.py" and python -I Tools/MakeBogKartRig.py
import bpy, bmesh, json, os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
ART = os.path.join(ROOT, 'Art', 'BogKart')
SRC = os.path.join(ART, 'Body_skinfront.fbx')
BACKUP = os.path.join(ART, 'Body_skinfront_withNose.fbx.bak')
if not os.path.exists(BACKUP):
    import shutil
    shutil.copyfile(SRC, BACKUP)

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=BACKUP if os.path.exists(BACKUP) else SRC)
ob = [o for o in bpy.context.scene.objects if o.type == 'MESH'][0]
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='DESELECT')
bpy.ops.object.mode_set(mode='OBJECT')
# Select the nose: connected component(s) whose centre is ahead of x = 1.25 m and below z = 0.2 m (world).
bm = bmesh.new()
bm.from_mesh(ob.data)
bm.verts.ensure_lookup_table()
M = ob.matrix_world
seen = set()
nose = set()
for v in bm.verts:
    if v.index in seen:
        continue
    stack, comp = [v], []
    seen.add(v.index)
    while stack:
        x = stack.pop()
        comp.append(x)
        for e in x.link_edges:
            o = e.other_vert(x)
            if o.index not in seen:
                seen.add(o.index)
                stack.append(o)
    pts = [M @ c.co for c in comp]
    cx = sum(p.x for p in pts) / len(pts)
    zmax = max(p.z for p in pts)
    if len(comp) > 100 and cx > 1.25 and zmax < 0.2:
        nose.update(c.index for c in comp)
bm.free()
print('NOSE verts', len(nose))
for v in ob.data.vertices:
    v.select = v.index in nose
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.separate(type='SELECTED')
bpy.ops.object.mode_set(mode='OBJECT')
objs = [o for o in bpy.context.scene.objects if o.type == 'MESH']
nose_ob = [o for o in objs if o != ob][0]


def export(o, name):
    bpy.ops.object.select_all(action='DESELECT')
    o.select_set(True)
    bpy.context.view_layer.objects.active = o
    bpy.ops.export_scene.fbx(filepath=os.path.join(ART, name + '.fbx'), use_selection=True,
                             apply_unit_scale=True, apply_scale_options='FBX_SCALE_ALL',
                             mesh_smooth_type='FACE', add_leaf_bones=False, bake_anim=False,
                             axis_forward='-Z', axis_up='Y')
    pts = [o.matrix_world @ v.co for v in o.data.vertices]
    print('EXPORT', name, len(o.data.polygons), [round(min(p[i] for p in pts), 3) for i in range(3)], [round(max(p[i] for p in pts), 3) for i in range(3)])
    return len(o.data.polygons)


export(ob, 'Body_skinfront')
tris = export(nose_ob, 'Nose_skinfront')
lay_path = os.path.join(ART, 'BogKartLayout.json')
lay = json.load(open(lay_path))
lay['parts'] = [p for p in lay['parts'] if p['asset'] != 'Nose_skinfront']
lay['parts'].append({'part': 'Nose', 'class': 'skinfront', 'asset': 'Nose_skinfront', 'tris': tris})
json.dump(lay, open(lay_path, 'w'), indent=2)
print('LAYOUT updated')
