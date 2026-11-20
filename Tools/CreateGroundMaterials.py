# Imports the CC0 ground textures (Art/Textures/CC0, ambientCG: Asphalt033, Grass005, Concrete034) and creates
# /Game/KartVisuals/M_KartGround plus one instance per surface (MI_Ground_Asphalt / Grass / Concrete).
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateGroundMaterials.py" -unattended -nosplash
#
# M_KartGround: world-projected (top-down XY, no UVs needed), PBR (colour, normal, roughness, AO) with an
# anti-tiling macro variation. Runtime parameters:
#   Tint (vector)          multiplies the albedo (kerb red / white paint, asphalt de-warming)
#   TileSize (cm)          size of one texture tile in the world
#   MacroScale, MacroAmount large-scale brightness variation (hides repetition)
#   NormalStrength, RoughnessScale
#   Wetness (0..1)         darkens porous albedo and pulls roughness toward a water film (0.08)
import unreal, os

ROOT = os.path.join(unreal.Paths.project_dir(), 'Art', 'Textures', 'CC0')
TEX = '/Game/KartVisuals/Ground'
mel = unreal.MaterialEditingLibrary
eal = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()

# --- textures ---
tasks = []
for set_id in ['Asphalt033', 'Grass005', 'Concrete034']:
    folder = os.path.join(ROOT, set_id)
    for f in sorted(os.listdir(folder)):
        t = unreal.AssetImportTask()
        t.filename = os.path.join(folder, f)
        t.destination_path = TEX
        t.destination_name = os.path.splitext(f)[0]
        t.automated = True
        t.replace_existing = True
        t.save = False
        tasks.append(t)
tools.import_asset_tasks(tasks)
for t in tasks:
    name = t.destination_name
    tex = unreal.load_asset(f'{TEX}/{name}')
    if tex is None:
        unreal.log_error('GROUND missing texture ' + name); continue
    if name.endswith('_N'):
        tex.set_editor_property('compression_settings', unreal.TextureCompressionSettings.TC_NORMALMAP)
        tex.set_editor_property('srgb', False)
    elif name.endswith('_R') or name.endswith('_AO'):
        tex.set_editor_property('compression_settings', unreal.TextureCompressionSettings.TC_MASKS)
        tex.set_editor_property('srgb', False)
    eal.save_asset(f'{TEX}/{name}')

# --- master material ---
full = '/Game/KartVisuals/M_KartGround'
if eal.does_asset_exist(full):
    eal.delete_asset(full)
mat = tools.create_asset('M_KartGround', '/Game/KartVisuals', unreal.Material, unreal.MaterialFactoryNew())

def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)
def scalar(name, v, x, y):
    n = node(unreal.MaterialExpressionScalarParameter, x, y)
    n.set_editor_property('parameter_name', name); n.set_editor_property('default_value', v); return n
def vector(name, v, x, y):
    n = node(unreal.MaterialExpressionVectorParameter, x, y)
    n.set_editor_property('parameter_name', name); n.set_editor_property('default_value', unreal.LinearColor(*v)); return n
def texparam(name, default, uv, x, y, sampler):
    n = node(unreal.MaterialExpressionTextureSampleParameter2D, x, y)
    n.set_editor_property('parameter_name', name)
    n.set_editor_property('texture', unreal.load_asset(default))
    n.set_editor_property('sampler_type', sampler)
    mel.connect_material_expressions(uv, '', n, 'UVs')
    return n
def custom(desc, code, out_type, ins, x, y):
    c = node(unreal.MaterialExpressionCustom, x, y)
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

wp = node(unreal.MaterialExpressionWorldPosition, -1800, 0)
mask = node(unreal.MaterialExpressionComponentMask, -1600, 0)
mask.set_editor_property('r', True); mask.set_editor_property('g', True)
mask.set_editor_property('b', False); mask.set_editor_property('a', False)
mel.connect_material_expressions(wp, '', mask, '')
tile = scalar('TileSize', 300.0, -1600, 150)
uv = node(unreal.MaterialExpressionDivide, -1400, 0)
mel.connect_material_expressions(mask, '', uv, 'A'); mel.connect_material_expressions(tile, '', uv, 'B')
mscale = scalar('MacroScale', 0.071, -1400, 150)
uvm = node(unreal.MaterialExpressionMultiply, -1250, 150)
mel.connect_material_expressions(uv, '', uvm, 'A'); mel.connect_material_expressions(mscale, '', uvm, 'B')

S = unreal.MaterialSamplerType
cC = texparam('ColorTex', f'{TEX}/T_Asphalt033_C', uv, -1100, -200, S.SAMPLERTYPE_COLOR)
cM = texparam('ColorTex', f'{TEX}/T_Asphalt033_C', uvm, -1100, 50, S.SAMPLERTYPE_COLOR)
cN = texparam('NormalTex', f'{TEX}/T_Asphalt033_N', uv, -1100, 300, S.SAMPLERTYPE_NORMAL)
cR = texparam('RoughnessTex', f'{TEX}/T_Asphalt033_R', uv, -1100, 550, S.SAMPLERTYPE_MASKS)
cA = texparam('AOTex', f'{TEX}/T_Asphalt033_AO', uv, -1100, 800, S.SAMPLERTYPE_MASKS)
tint = vector('Tint', (1, 1, 1, 1), -800, -300)
wet = scalar('Wetness', 0.0, -800, -150)
mamt = scalar('MacroAmount', 0.35, -800, -50)
nstr = scalar('NormalStrength', 1.0, -800, 400)
rsc = scalar('RoughnessScale', 1.0, -800, 600)
O = unreal.CustomMaterialOutputType
base = custom('GroundAlbedo', '''
float l = dot(M.rgb, float3(0.333, 0.333, 0.333));
float ref = 0.32;
float macro = lerp(1.0, saturate(l / ref), MacroAmount);
return C.rgb * Tint.rgb * macro * lerp(1.0, 0.5, Wet);
''', O.CMOT_FLOAT3, {'C': cC, 'M': cM, 'Tint': tint, 'MacroAmount': mamt, 'Wet': wet}, -500, -200)
rough = custom('GroundRoughness', 'return lerp(saturate(R.r * Scale), 0.08, Wet);', O.CMOT_FLOAT1,
               {'R': cR, 'Scale': rsc, 'Wet': wet}, -500, 500)
nrm = custom('GroundNormal', '''
float3 n = N.rgb;
float s = Strength * (1.0 - 0.6 * Wet);
return normalize(float3(n.xy * s, max(n.z, 0.01)));
''', O.CMOT_FLOAT3, {'N': cN, 'Strength': nstr, 'Wet': wet}, -500, 300)
P = unreal.MaterialProperty
mel.connect_material_property(base, '', P.MP_BASE_COLOR)
mel.connect_material_property(rough, '', P.MP_ROUGHNESS)
mel.connect_material_property(nrm, '', P.MP_NORMAL)
mel.connect_material_property(cA, 'R', P.MP_AMBIENT_OCCLUSION)
mel.recompile_material(mat)
eal.save_asset(full)

# --- instances ---
def instance(name, prefix, tile_cm, tint, rough_scale=1.0, normal=1.0, macro=0.35, ao=True):
    path = f'/Game/KartVisuals/{name}'
    if eal.does_asset_exist(path):
        eal.delete_asset(path)
    mi = tools.create_asset(name, '/Game/KartVisuals', unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mel.set_material_instance_parent(mi, mat)
    mel.set_material_instance_texture_parameter_value(mi, 'ColorTex', unreal.load_asset(f'{TEX}/T_{prefix}_C'))
    mel.set_material_instance_texture_parameter_value(mi, 'NormalTex', unreal.load_asset(f'{TEX}/T_{prefix}_N'))
    mel.set_material_instance_texture_parameter_value(mi, 'RoughnessTex', unreal.load_asset(f'{TEX}/T_{prefix}_R'))
    mel.set_material_instance_texture_parameter_value(mi, 'AOTex', unreal.load_asset(f'{TEX}/T_{prefix}_AO' if ao else '/Engine/EngineResources/WhiteSquareTexture'))
    mel.set_material_instance_scalar_parameter_value(mi, 'TileSize', tile_cm)
    mel.set_material_instance_scalar_parameter_value(mi, 'RoughnessScale', rough_scale)
    mel.set_material_instance_scalar_parameter_value(mi, 'NormalStrength', normal)
    mel.set_material_instance_scalar_parameter_value(mi, 'MacroAmount', macro)
    mel.set_material_instance_vector_parameter_value(mi, 'Tint', unreal.LinearColor(*tint))
    eal.save_asset(path)
    unreal.log_warning('GROUND created ' + path)

instance('MI_Ground_Asphalt', 'Asphalt033', 250.0, (0.62, 0.64, 0.72, 1.0), 1.0, 1.0, 0.45)
instance('MI_Ground_Grass', 'Grass005', 180.0, (0.85, 0.9, 0.8, 1.0), 1.0, 1.0, 0.5)
instance('MI_Ground_Concrete', 'Concrete034', 150.0, (1.0, 1.0, 1.0, 1.0), 1.0, 0.8, 0.2, ao=False)
unreal.log_warning('GROUND done')
