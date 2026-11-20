# Builds a used car tyre for trackside tyre walls (revolved profile) and exports Art/Track/BarrierTyre.fbx.
#   "C:/Program Files/Blender Foundation/Blender 4.3/blender.exe" -b -P Tools/MakeBarrierTyre.py
# Size (typical 195/65 R15 road tyre, ESTIMATE): outer diameter 0.62 m, bead (hole) diameter 0.38 m, width 0.20 m.
# Origin = centre of the tyre lying flat (axis = Z), so instances stack at 0.20 m spacing.
# UV: U = around the tyre (0..1, x8 repeats for tread detail), V = along the profile (0..1).
import bpy, bmesh, math, os

OUT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'Art', 'Track', 'BarrierTyre.fbx')
os.makedirs(os.path.dirname(OUT), exist_ok=True)

R_OUT, R_BEAD, HALF_W = 0.31, 0.19, 0.10
SEG = 48

# Profile (r, z), walked from the lower bead, out along the lower sidewall, over the tread, back in to the upper bead,
# then the inner liner (the hole is open: you see into the tyre from above, as on a real tyre wall).
prof = []
def arc(cx, cz, rad, a0, a1, n):
    for i in range(n + 1):
        a = math.radians(a0 + (a1 - a0) * i / n)
        prof.append((cx + rad * math.cos(a), cz + rad * math.sin(a)))
prof.append((R_BEAD, -0.075))                       # bead, lower
prof.append((R_BEAD + 0.02, -0.088))
prof.append((0.25, -0.095))                         # lower sidewall (slight bulge)
arc(R_OUT - 0.035, -HALF_W + 0.035, 0.035, -90, 0, 4)   # lower shoulder
# tread with 3 circumferential grooves
z = -HALF_W + 0.035
for gz in (-0.04, 0.0, 0.04):
    prof.append((R_OUT, gz - 0.008)); prof.append((R_OUT - 0.006, gz - 0.005))
    prof.append((R_OUT - 0.006, gz + 0.005)); prof.append((R_OUT, gz + 0.008))
arc(R_OUT - 0.035, HALF_W - 0.035, 0.035, 0, 90, 4)     # upper shoulder
prof.append((0.25, 0.095))
prof.append((R_BEAD + 0.02, 0.088))
prof.append((R_BEAD, 0.075))                        # bead, upper
prof.append((R_BEAD + 0.03, 0.05))                  # inner liner (inside the carcass, seen through the hole)
prof.append((R_BEAD + 0.03, -0.05))
prof.append((R_BEAD, -0.075))

# Arc-length V coordinate along the profile.
L = [0.0]
for i in range(1, len(prof)):
    L.append(L[-1] + math.dist(prof[i - 1], prof[i]))
V = [l / L[-1] for l in L]

me = bpy.data.meshes.new('BarrierTyre')
bm = bmesh.new()
uv_layer = bm.loops.layers.uv.new('UVMap')
rings = []
for s in range(SEG + 1):
    a = 2 * math.pi * s / SEG
    ring = []
    for (r, zz) in prof:
        ring.append(bm.verts.new((r * math.cos(a), r * math.sin(a), zz)) if s < SEG else None)
    rings.append(ring)
rings[SEG] = rings[0]
for s in range(SEG):
    for p in range(len(prof) - 1):
        vs = (rings[s][p], rings[s + 1][p], rings[s + 1][p + 1], rings[s][p + 1])
        f = bm.faces.new(vs)
        u0, u1 = 8.0 * s / SEG, 8.0 * (s + 1) / SEG
        for loop, uv in zip(f.loops, ((u0, V[p]), (u1, V[p]), (u1, V[p + 1]), (u0, V[p + 1]))):
            loop[uv_layer].uv = uv
bmesh.ops.recalc_face_normals(bm, faces=bm.faces)
bm.to_mesh(me)
bm.free()
for poly in me.polygons:
    poly.use_smooth = True

for o in list(bpy.data.objects):
    bpy.data.objects.remove(o, do_unlink=True)
ob = bpy.data.objects.new('BarrierTyre', me)
bpy.context.scene.collection.objects.link(ob)
bpy.ops.export_scene.fbx(filepath=OUT, use_selection=False, apply_unit_scale=True, apply_scale_options='FBX_SCALE_UNITS',
                         mesh_smooth_type='FACE', add_leaf_bones=False, bake_anim=False)
print('BARRIERTYRE exported', OUT, len(me.vertices), 'verts')
