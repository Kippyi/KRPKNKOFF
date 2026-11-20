# Splits Art/BogKart/Body_skinside.fbx (both sidepods in one mesh) into SidepodL_skinside.fbx / SidepodR_skinside.fbx so
# a side impact can knock one sidepod loose on the model (Milestone 8 damage). Loose parts go left / right by the sign of
# their centre y (kart frame, y left). Updates Art/BogKart/BogKartLayout.json (Body_skinside -> the two sidepods).
#   "C:/Program Files/Blender Foundation/Blender 4.3/blender.exe" -b -P Tools/SplitSidepods.py
# Then: UnrealEditor-Cmd ... -script="Tools/ImportPanels.py" and python -I Tools/MakeBogKartRig.py
import bpy, bmesh, json, os

ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
ART = os.path.join(ROOT, 'Art', 'BogKart')
SRC = os.path.join(ART, 'Body_skinside.fbx')

bpy.ops.wm.read_factory_settings(use_empty=True)
bpy.ops.import_scene.fbx(filepath=SRC)
ob = [o for o in bpy.context.scene.objects if o.type == 'MESH'][0]
bpy.context.view_layer.objects.active = ob
ob.select_set(True)
M = ob.matrix_world
bm = bmesh.new()
bm.from_mesh(ob.data)
bm.verts.ensure_lookup_table()
seen, left = set(), set()
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
    cy = sum((M @ c.co).y for c in comp) / len(comp)
    if cy > 0.0:
        left.update(c.index for c in comp)
bm.free()
print('LEFT verts', len(left), 'of', len(ob.data.vertices))
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.select_all(action='DESELECT')
bpy.ops.object.mode_set(mode='OBJECT')
for v in ob.data.vertices:
    v.select = v.index in left
bpy.ops.object.mode_set(mode='EDIT')
bpy.ops.mesh.separate(type='SELECTED')
bpy.ops.object.mode_set(mode='OBJECT')
left_ob = [o for o in bpy.context.scene.objects if o.type == 'MESH' and o != ob][0]


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


tl = export(left_ob, 'SidepodL_skinside')
tr = export(ob, 'SidepodR_skinside')
lay_path = os.path.join(ART, 'BogKartLayout.json')
lay = json.load(open(lay_path))
lay['parts'] = [p for p in lay['parts'] if p['asset'] not in ('Body_skinside', 'SidepodL_skinside', 'SidepodR_skinside')]
lay['parts'].append({'part': 'SidepodL', 'class': 'skinside', 'asset': 'SidepodL_skinside', 'tris': tl})
lay['parts'].append({'part': 'SidepodR', 'class': 'skinside', 'asset': 'SidepodR_skinside', 'tris': tr})
json.dump(lay, open(lay_path, 'w'), indent=2)
print('LAYOUT updated')
