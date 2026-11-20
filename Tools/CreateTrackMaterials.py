# Realism pass 4: track asphalt and grass masters (reuse the CC0 ambientCG textures imported by CreateGroundMaterials.py).
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateTrackMaterials.py" -unattended -nosplash
#
# M_KartAsphalt + MI_Track_Asphalt: world-projected Asphalt033 plus wear masks from the procedural asphalt mesh
#   (AKartTestTrackActor::BuildTrack):  UV0.x = 0..1 across the track,  UV0.y = sample index along it (x SampleSpacing m),
#   vertex colour R = racing-line position in UV.x space, G = half width / 10 m, B = corner / braking-zone intensity.
#   -> rubbered racing line (darker, smoother, streaky), tyre marks in braking zones, dusty light edges.
# M_KartGrass + MI_Track_Grass: Grass005 sampled at two scales / rotations (no visible tiling) with a large-scale
#   hue, dryness and brightness variation (procedural fBm), desaturated toward real turf.
# Both keep the runtime parameters the game sets: Tint (vector) and Wetness (0..1).
import unreal

TEX = '/Game/KartVisuals/Ground'
mel = unreal.MaterialEditingLibrary
eal = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()
O = unreal.CustomMaterialOutputType
S = unreal.MaterialSamplerType
P = unreal.MaterialProperty

# HLSL value noise / fBm. UE custom nodes allow a struct with methods declared inline.
NOISE = '''
struct KN {
  float h(float2 p) { p = frac(p * float2(0.1031, 0.1030)); p += dot(p, p.yx + 33.33); return frac((p.x + p.y) * p.x); }
  float v(float2 p) { float2 i = floor(p); float2 f = frac(p); f = f * f * (3.0 - 2.0 * f);
    return lerp(lerp(h(i), h(i + float2(1, 0)), f.x), lerp(h(i + float2(0, 1)), h(i + float2(1, 1)), f.x), f.y); }
  float fbm(float2 p) { float s = 0.0; float a = 0.5; for (int k = 0; k < 4; k++) { s += a * v(p); p = p * 2.03 + 17.1; a *= 0.5; } return s / 0.9375; }
};
KN kn;
'''


def new_material(name):
    full = f'/Game/KartVisuals/{name}'
    if eal.does_asset_exist(full):
        eal.delete_asset(full)
    return tools.create_asset(name, '/Game/KartVisuals', unreal.Material, unreal.MaterialFactoryNew()), full


class G:
    def __init__(self, mat):
        self.mat = mat

    def node(self, cls, x, y):
        return mel.create_material_expression(self.mat, cls, x, y)

    def scalar(self, name, v, x, y):
        n = self.node(unreal.MaterialExpressionScalarParameter, x, y)
        n.set_editor_property('parameter_name', name); n.set_editor_property('default_value', v); return n

    def vector(self, name, v, x, y):
        n = self.node(unreal.MaterialExpressionVectorParameter, x, y)
        n.set_editor_property('parameter_name', name); n.set_editor_property('default_value', unreal.LinearColor(*v)); return n

    def tex(self, name, default, uv, x, y, sampler):
        n = self.node(unreal.MaterialExpressionTextureSampleParameter2D, x, y)
        n.set_editor_property('parameter_name', name)
        n.set_editor_property('texture', unreal.load_asset(default))
        n.set_editor_property('sampler_type', sampler)
        mel.connect_material_expressions(uv, '', n, 'UVs')
        return n

    def custom(self, desc, code, out_type, ins, x, y):
        c = self.node(unreal.MaterialExpressionCustom, x, y)
        c.set_editor_property('output_type', out_type)
        c.set_editor_property('description', desc)
        c.set_editor_property('code', code)
        lst = []
        for k in ins:
            ci = unreal.CustomInput(); ci.set_editor_property('input_name', k); lst.append(ci)
        c.set_editor_property('inputs', lst)
        for k, v in ins.items():
            mel.connect_material_expressions(v, '', c, k)
        return c

    def world_xy_m(self, x, y):
        """World XY in metres."""
        wp = self.node(unreal.MaterialExpressionWorldPosition, x, y)
        return self.custom('WorldXYm', 'return W.xy * 0.01;', O.CMOT_FLOAT2, {'W': wp}, x + 200, y)


def make_instance(name, parent, params, vectors=None):
    path = f'/Game/KartVisuals/{name}'
    if eal.does_asset_exist(path):
        eal.delete_asset(path)
    mi = tools.create_asset(name, '/Game/KartVisuals', unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mel.set_material_instance_parent(mi, parent)
    for k, v in params.items():
        mel.set_material_instance_scalar_parameter_value(mi, k, v)
    for k, v in (vectors or {}).items():
        mel.set_material_instance_vector_parameter_value(mi, k, unreal.LinearColor(*v))
    eal.save_asset(path)
    unreal.log_warning('TRACKMAT created ' + path)


# ------------------------------------------------------------------ asphalt
mat, full = new_material('M_KartAsphalt')
g = G(mat)
xy = g.world_xy_m(-2200, 0)
uv = g.custom('AsphaltUV', 'return XY / (Tile * 0.01);', O.CMOT_FLOAT2, {'XY': xy, 'Tile': g.scalar('TileSize', 250.0, -2200, 150)}, -1900, 0)
uvm = g.custom('MacroUV', 'return UV * 0.071;', O.CMOT_FLOAT2, {'UV': uv}, -1700, 150)
cC = g.tex('ColorTex', f'{TEX}/T_Asphalt033_C', uv, -1400, -300, S.SAMPLERTYPE_COLOR)
cM = g.tex('MacroTex', f'{TEX}/T_Asphalt033_C', uvm, -1400, -50, S.SAMPLERTYPE_COLOR)
cN = g.tex('NormalTex', f'{TEX}/T_Asphalt033_N', uv, -1400, 200, S.SAMPLERTYPE_NORMAL)
cR = g.tex('RoughnessTex', f'{TEX}/T_Asphalt033_R', uv, -1400, 450, S.SAMPLERTYPE_MASKS)
cA = g.tex('AOTex', f'{TEX}/T_Asphalt033_AO', uv, -1400, 700, S.SAMPLERTYPE_MASKS)
vc = g.node(unreal.MaterialExpressionVertexColor, -1400, -700)
tc = g.node(unreal.MaterialExpressionTextureCoordinate, -1400, -550)
# Masks: x = rubber (racing line), y = tyre marks, z = edge dust, w = off-line grime noise.
masks = g.custom('AsphaltWear', NOISE + '''
float hw = max(VC.g * 10.0, 1.0);
float w = 2.0 * hw;
float lat = UV.x * w;                       // m from the right edge
float along = UV.y * Spacing;               // m along the track
float dLine = abs(UV.x - VC.r) * w;         // m from the racing line
float edge = min(lat, w - lat);             // m from the nearest edge
// Rubber: a ~1.6 m band on the line, streaky along the direction of travel, heavier in corners.
float streak = kn.fbm(float2(lat * 7.0, along * 0.06));
float band = exp(-pow(dLine / LineWidth, 2.0));
float rubber = saturate(band * (0.55 + 0.45 * streak) * Rubber * (0.65 + 0.35 * VC.b));
// Tyre marks: thin dark streaks near the line in braking zones / corners, broken up along the track.
float m1 = kn.v(float2(lat * 2.6 + kn.v(float2(along * 0.05, 3.1)) * 2.0, along * 0.035));
float marks = smoothstep(0.60, 0.80, m1) * smoothstep(0.30, 0.90, kn.v(float2(along * 0.09, lat * 0.5 + 7.0)));
marks *= VC.b * exp(-pow(dLine / (LineWidth * 1.6), 2.0)) * Marks;
// Edges: unused, dusty and lighter, ragged along the track.
float rag = 0.35 + 0.65 * kn.fbm(float2(along * 0.25, lat * 0.8 + 11.0));
float dust = (1.0 - smoothstep(0.0, EdgeWidth * rag, edge)) * EdgeDust;
float grime = kn.fbm(float2(lat * 0.9, along * 0.12) + 41.0);
return float4(rubber, saturate(marks), saturate(dust), grime);
''', O.CMOT_FLOAT4, {
    'VC': vc, 'UV': tc,
    'Spacing': g.scalar('SampleSpacing', 0.75, -1700, -800),
    'LineWidth': g.scalar('LineWidth', 0.85, -1700, -700),
    'Rubber': g.scalar('RubberAmount', 0.8, -1700, -600),
    'Marks': g.scalar('TyreMarks', 0.7, -1700, -500),
    'EdgeWidth': g.scalar('EdgeWidth', 0.9, -1700, -400),
    'EdgeDust': g.scalar('EdgeDust', 0.6, -1700, -300),
}, -1000, -650)
tint = g.vector('Tint', (1, 1, 1, 1), -1000, -350)
wet = g.scalar('Wetness', 0.0, -1000, -250)
base = g.custom('AsphaltAlbedo', '''
float l = dot(Mc.rgb, float3(0.333, 0.333, 0.333));
float macro = lerp(1.0, saturate(l / 0.32), MacroAmount);
float3 c = C.rgb * BaseTint.rgb * Tint.rgb * macro;
c *= lerp(0.92, 1.06, Mk.w);                            // off-line grime / patches
c *= lerp(1.0, RubberDark, Mk.x);                       // rubbered line
c *= lerp(1.0, 0.45, Mk.y);                             // tyre marks
c = lerp(c, c * 1.25 + DustColor.rgb * 0.05, Mk.z);     // dusty edges
return c * lerp(1.0, 0.5, Wet);
''', O.CMOT_FLOAT3, {
    'C': cC, 'Mc': cM, 'Mk': masks, 'Tint': tint, 'Wet': wet,
    'BaseTint': g.vector('BaseTint', (0.40, 0.41, 0.45, 1.0), -700, -450),
    'MacroAmount': g.scalar('MacroAmount', 0.45, -700, -350),
    'RubberDark': g.scalar('RubberDarken', 0.55, -700, -250),
    'DustColor': g.vector('DustColor', (0.55, 0.48, 0.38, 1.0), -700, -150),
}, -400, -350)
rough = g.custom('AsphaltRoughness', '''
float r = saturate(R.r * Scale);
r = lerp(r, r * 0.82, Mk.x);       // rubber fills the texture: a faint sheen on the line
r = lerp(r, 0.95, Mk.z * 0.6);     // dust
return lerp(r, 0.08, Wet);
''', O.CMOT_FLOAT1, {'R': cR, 'Mk': masks, 'Wet': wet, 'Scale': g.scalar('RoughnessScale', 1.0, -700, 550)}, -400, 450)
nrm = g.custom('AsphaltNormal', '''
float s = Strength * (1.0 - 0.6 * Wet) * (1.0 - 0.45 * Mk.x) * (1.0 - 0.3 * Mk.z);
return normalize(float3(N.xy * s, max(N.z, 0.01)));
''', O.CMOT_FLOAT3, {'N': cN, 'Mk': masks, 'Wet': wet, 'Strength': g.scalar('NormalStrength', 1.0, -700, 300)}, -400, 250)
mel.connect_material_property(base, '', P.MP_BASE_COLOR)
mel.connect_material_property(rough, '', P.MP_ROUGHNESS)
mel.connect_material_property(nrm, '', P.MP_NORMAL)
mel.connect_material_property(cA, 'R', P.MP_AMBIENT_OCCLUSION)
mel.recompile_material(mat)
eal.save_asset(full)
make_instance('MI_Track_Asphalt', mat, {})

# ------------------------------------------------------------------ grass
mat, full = new_material('M_KartGrass')
g = G(mat)
xy = g.world_xy_m(-2200, 0)
tile = g.scalar('TileSize', 180.0, -2200, 150)
uvA = g.custom('GrassUVA', 'return XY / (Tile * 0.01);', O.CMOT_FLOAT2, {'XY': xy, 'Tile': tile}, -1900, -100)
uvB = g.custom('GrassUVB', '''
float2 p = XY / (Tile * 0.01 * 2.7);
float s = 0.6018, c = 0.7986;                  // 37 deg
return float2(c * p.x - s * p.y, s * p.x + c * p.y) + 0.37;
''', O.CMOT_FLOAT2, {'XY': xy, 'Tile': tile}, -1900, 150)
aC = g.tex('ColorTex', f'{TEX}/T_Grass005_C', uvA, -1400, -400, S.SAMPLERTYPE_COLOR)
bC = g.tex('ColorTexB', f'{TEX}/T_Grass005_C', uvB, -1400, -150, S.SAMPLERTYPE_COLOR)
aN = g.tex('NormalTex', f'{TEX}/T_Grass005_N', uvA, -1400, 100, S.SAMPLERTYPE_NORMAL)
aR = g.tex('RoughnessTex', f'{TEX}/T_Grass005_R', uvA, -1400, 350, S.SAMPLERTYPE_MASKS)
aA = g.tex('AOTex', f'{TEX}/T_Grass005_AO', uvA, -1400, 600, S.SAMPLERTYPE_MASKS)
var = g.custom('GrassVariation', NOISE + '''
float big = kn.fbm(XY / 38.0);          // drainage / soil patches
float mid = kn.fbm(XY / 7.0 + 19.0);    // clumps
float dry = smoothstep(0.45, 0.80, big * 0.7 + mid * 0.3);
float bright = lerp(0.72, 1.15, mid) * lerp(0.9, 1.05, big);
float mow = 0.5 + 0.5 * sin(dot(XY, float2(0.6, 0.8)) * 6.2832 / MowStripe);  // faint mowing stripes
return float4(dry, bright, mow, big);
''', O.CMOT_FLOAT4, {'XY': xy, 'MowStripe': g.scalar('MowStripe', 3.0, -1700, -700)}, -1000, -650)
tint = g.vector('Tint', (1, 1, 1, 1), -1000, -350)
wet = g.scalar('Wetness', 0.0, -1000, -250)
# Verge (shoulder strip, UV.x 0 at the asphalt / kerb edge -> 1 where it meets the terrain, UV.y = sample index):
# worn to dirt next to the track where karts run wide, ragged along the track. VergeWear = 0 on the terrain.
vtc = g.node(unreal.MaterialExpressionTextureCoordinate, -1400, -900)
dirt = g.custom('VergeDirt', NOISE + '''
float along = UV.y * 0.75;
float rag = kn.fbm(float2(along * 0.35, 3.0));
float clump = kn.fbm(XY * 1.7 + 5.0);
float reach = lerp(0.03, 0.22, rag * (0.6 + 0.4 * Corner));
float d = 1.0 - smoothstep(reach * 0.4, reach, UV.x + (clump - 0.5) * 0.10);
float patchy = smoothstep(0.30, 0.65, kn.fbm(XY * 0.9 + 21.0));   // grass survives in patches
return saturate(d * lerp(0.35, 0.9, patchy) * Wear);
''', O.CMOT_FLOAT1, {'UV': vtc, 'XY': xy, 'Wear': g.scalar('VergeWear', 0.0, -1700, -900),
                     'Corner': g.scalar('VergeCorner', 0.7, -1700, -1000)}, -1000, -900)
base = g.custom('GrassAlbedo', '''
float3 c = lerp(A.rgb, B.rgb, 0.45);
float l = dot(c, float3(0.299, 0.587, 0.114));
c = lerp(l.xxx, c, Saturation);
c *= Lush.rgb;
c = lerp(c, l.xxx * Dry.rgb, V.x * DryAmount);
c *= V.y * lerp(1.0 - MowAmount, 1.0 + MowAmount, V.z);
float3 soil = DirtTint.rgb * lerp(0.75, 1.15, V.w) * lerp(0.8, 1.1, l / 0.25);
c = lerp(c, soil, D);
return c * Tint.rgb * lerp(1.0, lerp(0.7, 0.55, D), Wet);
''', O.CMOT_FLOAT3, {
    'A': aC, 'B': bC, 'V': var, 'Tint': tint, 'Wet': wet, 'D': dirt,
    'DirtTint': g.vector('DirtTint', (0.13, 0.105, 0.078, 1.0), -700, -650),
    'Saturation': g.scalar('Saturation', 0.92, -700, -550),
    'Lush': g.vector('LushTint', (0.40, 0.47, 0.28, 1.0), -700, -450),
    'Dry': g.vector('DryTint', (0.92, 0.82, 0.55, 1.0), -700, -350),
    'DryAmount': g.scalar('DryAmount', 0.35, -700, -250),
    'MowAmount': g.scalar('MowAmount', 0.05, -700, -150),
}, -400, -350)
rough = g.custom('GrassRoughness', 'return lerp(lerp(saturate(R.r), 0.92, D), lerp(0.35, 0.2, D), Wet);', O.CMOT_FLOAT1, {'R': aR, 'Wet': wet, 'D': dirt}, -400, 450)
nrm = g.custom('GrassNormal', 'return normalize(float3(N.xy * Strength * (1.0 - 0.6 * D), max(N.z, 0.01)));', O.CMOT_FLOAT3,
               {'N': aN, 'D': dirt, 'Strength': g.scalar('NormalStrength', 1.0, -700, 250)}, -400, 250)
mel.connect_material_property(base, '', P.MP_BASE_COLOR)
mel.connect_material_property(rough, '', P.MP_ROUGHNESS)
mel.connect_material_property(nrm, '', P.MP_NORMAL)
mel.connect_material_property(aA, 'R', P.MP_AMBIENT_OCCLUSION)
mel.recompile_material(mat)
eal.save_asset(full)
make_instance('MI_Track_Grass', mat, {})
make_instance('MI_Track_Verge', mat, {'VergeWear': 1.0})

# ------------------------------------------------------------------ kerbs
# Kerb mesh UVs (AKartTestTrackActor::BuildTrack): UV.x = metres from the inner (track-side) edge, UV.y = metres along.
# Paint (Tint, set by the game) over concrete, rubber smudged on the inner half where the tyres ride, chipped paint.
mat, full = new_material('M_KartKerb')
g = G(mat)
xy = g.world_xy_m(-2200, 0)
uvk = g.custom('KerbUV', 'return XY / 1.5;', O.CMOT_FLOAT2, {'XY': xy}, -1900, 0)
kC = g.tex('ColorTex', f'{TEX}/T_Concrete034_C', uvk, -1400, -300, S.SAMPLERTYPE_COLOR)
kN = g.tex('NormalTex', f'{TEX}/T_Concrete034_N', uvk, -1400, 0, S.SAMPLERTYPE_NORMAL)
kR = g.tex('RoughnessTex', f'{TEX}/T_Concrete034_R', uvk, -1400, 300, S.SAMPLERTYPE_MASKS)
ktc = g.node(unreal.MaterialExpressionTextureCoordinate, -1400, -600)
kmask = g.custom('KerbWear', NOISE + '''
float lat = UV.x;  float along = UV.y;
float streak = kn.fbm(float2(lat * 9.0, along * 0.35));
float blot = kn.fbm(float2(along * 0.6, lat * 2.0) + 9.0);
float rubber = exp(-lat / RubberReach) * smoothstep(0.35, 0.75, streak * 0.6 + blot * 0.4) * Rubber;
float chips = smoothstep(0.70, 0.78, kn.fbm(XY * 6.0 + 3.0)) * Chips;
return float4(saturate(rubber), chips, streak, 0);
''', O.CMOT_FLOAT4, {'UV': ktc, 'XY': xy,
                     'RubberReach': g.scalar('RubberReach', 0.35, -1700, -700),
                     'Rubber': g.scalar('RubberAmount', 0.85, -1700, -600),
                     'Chips': g.scalar('ChipAmount', 0.6, -1700, -500)}, -1000, -600)
tint = g.vector('Tint', (1, 1, 1, 1), -1000, -300)
wet = g.scalar('Wetness', 0.0, -1000, -200)
kbase = g.custom('KerbAlbedo', '''
float l = dot(C.rgb, float3(0.333, 0.333, 0.333));
float3 paint = Tint.rgb * lerp(0.82, 1.0, saturate(l / 0.5));
float3 c = lerp(paint, C.rgb * 0.8, M.y);               // chipped to concrete
c = lerp(c, float3(0.035, 0.034, 0.033), M.x * 0.85);   // rubber
return c * lerp(1.0, 0.6, Wet);
''', O.CMOT_FLOAT3, {'C': kC, 'M': kmask, 'Tint': tint, 'Wet': wet}, -400, -300)
krough = g.custom('KerbRoughness', '''
float r = lerp(0.55, saturate(R.r), M.y);   // paint is smoother than bare concrete
r = lerp(r, 0.7, M.x);
return lerp(r, 0.06, Wet);                  // wet paint is very slippery-looking
''', O.CMOT_FLOAT1, {'R': kR, 'M': kmask, 'Wet': wet}, -400, 300)
knrm = g.custom('KerbNormal', 'return normalize(float3(N.xy * lerp(0.35, 1.0, M.y), max(N.z, 0.01)));', O.CMOT_FLOAT3,
                {'N': kN, 'M': kmask}, -400, 100)
mel.connect_material_property(kbase, '', P.MP_BASE_COLOR)
mel.connect_material_property(krough, '', P.MP_ROUGHNESS)
mel.connect_material_property(knrm, '', P.MP_NORMAL)
mel.recompile_material(mat)
eal.save_asset(full)
make_instance('MI_Track_Kerb', mat, {})

# ------------------------------------------------------------------ barrier tyre mesh + material
import os
fbx = os.path.join(unreal.Paths.project_dir(), 'Art', 'Track', 'BarrierTyre.fbx')
opts = unreal.FbxImportUI()
opts.import_mesh = True
opts.import_as_skeletal = False
opts.import_materials = False
opts.import_textures = False
opts.import_animations = False
opts.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
sm = opts.static_mesh_import_data
sm.combine_meshes = True
sm.auto_generate_collision = False
sm.generate_lightmap_u_vs = False
sm.normal_import_method = unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
sm.convert_scene = True
t = unreal.AssetImportTask()
t.filename = fbx
t.destination_path = '/Game/KartVisuals/Track'
t.destination_name = 'SM_BarrierTyre'
t.automated = True
t.replace_existing = True
t.save = True
t.options = opts
tools.import_asset_tasks([t])
mesh = unreal.load_asset('/Game/KartVisuals/Track/SM_BarrierTyre')
if mesh:
    b = mesh.get_bounding_box()
    unreal.log_warning('BARRIER mesh min(%.1f %.1f %.1f) max(%.1f %.1f %.1f)' % (b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z))
else:
    unreal.log_error('BARRIER mesh import failed')

# Per-instance custom data: [0] paint (0 = bare rubber, 1 = white, 2 = red), [1] random 0..1 (fade / dust).
mat, full = new_material('M_BarrierTyre')
mat.set_editor_property('used_with_instanced_static_meshes', True)
g = G(mat)
def pic(idx, x, y):
    n = g.node(unreal.MaterialExpressionPerInstanceCustomData, x, y)
    n.set_editor_property('data_index', idx)
    return n
paint = pic(0, -1400, -300)
rnd = pic(1, -1400, -150)
tuv = g.node(unreal.MaterialExpressionTextureCoordinate, -1400, -450)
nws = g.node(unreal.MaterialExpressionVertexNormalWS, -1400, 0)
wpos = g.world_xy_m(-1400, 150)
twet = g.scalar('Wetness', 0.0, -1700, -100)
tb = g.custom('TyreAlbedo', NOISE + '''
float n = kn.fbm(UV * float2(6.0, 14.0) + Rnd * 37.0);
float3 rubber = float3(0.032, 0.031, 0.030) * lerp(0.8, 1.35, n) * lerp(0.9, 1.25, Rnd);   // sun-faded
float3 white = float3(0.62, 0.62, 0.60) * lerp(0.65, 1.0, n);
float3 red = float3(0.50, 0.035, 0.03) * lerp(0.6, 1.0, n);
float3 c = Paint < 0.5 ? rubber : (Paint < 1.5 ? white : red);
// paint worn through where tyres rub (tread) and dust settled on upward faces
float wear = smoothstep(0.55, 0.85, kn.fbm(UV * float2(20.0, 5.0) + 4.0)) * step(0.5, Paint);
c = lerp(c, rubber, wear);
float up = saturate(Nw.z);
c = lerp(c, float3(0.16, 0.14, 0.11), up * up * lerp(0.25, 0.55, kn.fbm(XY * 3.0)));
return c * lerp(1.0, 0.6, Wet);
''', O.CMOT_FLOAT3, {'UV': tuv, 'Paint': paint, 'Rnd': rnd, 'Nw': nws, 'XY': wpos,
                     'Wet': twet}, -800, -300)
tr = g.custom('TyreRoughness', 'return lerp(Paint > 0.5 ? 0.6 : 0.82, 0.25, Wet);', O.CMOT_FLOAT1,
              {'Paint': paint, 'Wet': twet}, -800, 100)
mel.connect_material_property(tb, '', P.MP_BASE_COLOR)
mel.connect_material_property(tr, '', P.MP_ROUGHNESS)
mel.recompile_material(mat)
eal.save_asset(full)
unreal.log_warning('TRACKMAT done')

