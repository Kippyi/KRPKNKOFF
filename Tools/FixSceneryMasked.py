# Poly Haven's glTF exports use JPG colour maps (no alpha) and leave the cut-out maps out, so the importer brings
# leaves and chain-link in as opaque instances of M_GLTF. This:
#   1. imports the separate alpha maps (downloaded next to each model: textures/<id>_<map>_alpha_2k.png),
#   2. creates /Game/Scenery/M_SceneryMasked: masked, two-sided, glTF conventions (OpenGL normals, ARM map:
#      G = roughness, B = metallic), opacity = OpacityTexture.R, UVScale (the wire's KHR_texture_transform scale 4),
#   3. re-points the leaf / wire slots to new instances of it, and restores any slot pointing at an older
#      *_Masked instance that is not in the table (the lamp / floodlight glass of a previous run).
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/FixSceneryMasked.py" -unattended -nosplash
import unreal, os

mel = unreal.MaterialEditingLibrary
eal = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()
SRC = os.path.join(unreal.Paths.project_dir(), 'Art', 'Models', 'PolyHaven')
MASTER = '/Game/Scenery/M_SceneryMasked'
# original material -> (asset id, alpha map file, UV scale)
TABLE = {
    '/Game/Scenery/jacaranda_tree/jacaranda_tree_2k/Materials/jacaranda_tree_leaves': ('jacaranda_tree', 'jacaranda_tree_leaves_alpha_2k.png', 1.0),
    '/Game/Scenery/tree_small_02/tree_small_02_2k/Materials/tree_small_02_leaves': ('tree_small_02', 'tree_small_02_leaves_alpha_2k.png', 1.0),
    '/Game/Scenery/fir_sapling_medium/fir_sapling_medium_2k/Materials/fir_sapling_medium_twigs': ('fir_sapling_medium', 'fir_sapling_medium_twigs_alpha_2k.png', 1.0),
    '/Game/Scenery/modular_chainlink_fence/modular_chainlink_fence_2k/Materials/modular_chainlink_fence_wire': ('modular_chainlink_fence', 'modular_chainlink_fence_wire_alpha_2k.png', 4.0),
}

# --- master ---
if eal.does_asset_exist(MASTER):
    eal.delete_asset(MASTER)
mat = tools.create_asset('M_SceneryMasked', '/Game/Scenery', unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property('blend_mode', unreal.BlendMode.BLEND_MASKED)
mat.set_editor_property('two_sided', True)
mat.set_editor_property('opacity_mask_clip_value', 0.4)
def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)
S = unreal.MaterialSamplerType
P = unreal.MaterialProperty
uv = node(unreal.MaterialExpressionTextureCoordinate, -1300, 0)
uvs = node(unreal.MaterialExpressionScalarParameter, -1300, 150)
uvs.set_editor_property('parameter_name', 'UVScale'); uvs.set_editor_property('default_value', 1.0)
uvm = node(unreal.MaterialExpressionMultiply, -1100, 0)
mel.connect_material_expressions(uv, '', uvm, 'A'); mel.connect_material_expressions(uvs, '', uvm, 'B')
def tex(name, x, y, sampler, default):
    n = node(unreal.MaterialExpressionTextureSampleParameter2D, x, y)
    n.set_editor_property('parameter_name', name)
    n.set_editor_property('sampler_type', sampler)
    n.set_editor_property('texture', unreal.load_asset(default))
    mel.connect_material_expressions(uvm, '', n, 'UVs')
    return n
white = '/Engine/EngineResources/WhiteSquareTexture'
base = tex('BaseColorTexture', -800, -300, S.SAMPLERTYPE_COLOR, white)
arm = tex('MetallicRoughnessTexture', -800, 0, S.SAMPLERTYPE_LINEAR_COLOR, white)
opa = tex('OpacityTexture', -800, 250, S.SAMPLERTYPE_LINEAR_COLOR, white)
nrm = tex('NormalTexture', -800, 500, S.SAMPLERTYPE_NORMAL, '/Engine/EngineMaterials/DefaultNormal')
mel.connect_material_property(base, 'RGB', P.MP_BASE_COLOR)
mel.connect_material_property(arm, 'G', P.MP_ROUGHNESS)
mel.connect_material_property(arm, 'B', P.MP_METALLIC)
# Distance fade (FadeStart..FadeEnd cm, camera depth): fine wire aliases into dark sheets once it is sub-pixel, so
# the chain-link mask fades out with distance (real chain-link is near invisible at range). Leaves keep the default
# (no fade).
depth = node(unreal.MaterialExpressionPixelDepth, -800, 800)
fs = node(unreal.MaterialExpressionScalarParameter, -800, 900)
fs.set_editor_property('parameter_name', 'FadeStart'); fs.set_editor_property('default_value', 1.0e7)
fe = node(unreal.MaterialExpressionScalarParameter, -800, 1000)
fe.set_editor_property('parameter_name', 'FadeEnd'); fe.set_editor_property('default_value', 1.1e7)
fade = node(unreal.MaterialExpressionCustom, -500, 300)
fade.set_editor_property('output_type', unreal.CustomMaterialOutputType.CMOT_FLOAT1)
fade.set_editor_property('code', 'return A * (1.0 - saturate((D - S) / max(E - S, 1.0)));')
ins = []
for k in ('A', 'D', 'S', 'E'):
    ci = unreal.CustomInput(); ci.set_editor_property('input_name', k); ins.append(ci)
fade.set_editor_property('inputs', ins)
mel.connect_material_expressions(opa, 'R', fade, 'A')
mel.connect_material_expressions(depth, '', fade, 'D')
mel.connect_material_expressions(fs, '', fade, 'S')
mel.connect_material_expressions(fe, '', fade, 'E')
mel.connect_material_property(fade, '', P.MP_OPACITY_MASK)
flip = node(unreal.MaterialExpressionMultiply, -400, 500)
const = node(unreal.MaterialExpressionConstant3Vector, -600, 650)
const.set_editor_property('constant', unreal.LinearColor(1, -1, 1, 1))   # OpenGL -> DirectX normal (flip green)
mel.connect_material_expressions(nrm, 'RGB', flip, 'A')
mel.connect_material_expressions(const, '', flip, 'B')
mel.connect_material_property(flip, '', P.MP_NORMAL)
mel.recompile_material(mat)
eal.save_asset(MASTER)

# --- instances ---
replaced = {}
for orig, (aid, alpha_file, uv_scale) in TABLE.items():
    src = unreal.load_asset(orig)
    if not src:
        unreal.log_error('MASKED missing ' + orig); continue
    folder = orig.rsplit('/', 2)[0] + '/Textures'
    t = unreal.AssetImportTask()
    t.filename = os.path.join(SRC, aid, 'textures', alpha_file)
    t.destination_path = folder
    t.destination_name = os.path.splitext(alpha_file)[0]
    t.automated = True; t.replace_existing = True; t.save = False
    tools.import_asset_tasks([t])
    alpha = unreal.load_asset(f'{folder}/{t.destination_name}')
    alpha.set_editor_property('srgb', False)
    # TC_DEFAULT (linear) matches the material's Linear Color sampler; a Grayscale texture there is a sampler-type
    # mismatch, which silently falls back to the default (white = fully opaque) texture.
    alpha.set_editor_property('compression_settings', unreal.TextureCompressionSettings.TC_DEFAULT)
    eal.save_asset(alpha.get_path_name())
    texs = {str(v.parameter_info.name): v.parameter_value for v in src.get_editor_property('texture_parameter_values')}
    path = orig + '_Masked'
    if eal.does_asset_exist(path):
        eal.delete_asset(path)
    mi = tools.create_asset(path.rsplit('/', 1)[1], path.rsplit('/', 1)[0], unreal.MaterialInstanceConstant, unreal.MaterialInstanceConstantFactoryNew())
    mel.set_material_instance_parent(mi, mat)
    for k in ('BaseColorTexture', 'MetallicRoughnessTexture', 'NormalTexture'):
        if texs.get(k):
            mel.set_material_instance_texture_parameter_value(mi, k, texs[k])
    mel.set_material_instance_texture_parameter_value(mi, 'OpacityTexture', alpha)
    mel.set_material_instance_scalar_parameter_value(mi, 'UVScale', uv_scale)
    if 'chainlink' in orig:
        mel.set_material_instance_scalar_parameter_value(mi, 'FadeStart', 1500.0)
        mel.set_material_instance_scalar_parameter_value(mi, 'FadeEnd', 3000.0)
    eal.save_asset(path)
    replaced[src.get_path_name()] = mi
    unreal.log_warning('MASKED instance %s (alpha %s, uv x%.0f)' % (path, alpha.get_name(), uv_scale))

# --- re-point mesh slots ---
for p in eal.list_assets('/Game/Scenery', recursive=True):
    m = unreal.load_asset(p)
    if not isinstance(m, unreal.StaticMesh):
        continue
    changed = False
    for i, sm in enumerate(m.static_materials):
        mi = sm.material_interface
        if not mi:
            continue
        name = mi.get_path_name()
        orig = name.replace('_Masked', '') if '_Masked' in name else name
        if orig in replaced:
            if mi != replaced[orig]:
                m.set_material(i, replaced[orig]); changed = True
        elif '_Masked' in name:   # stale instance from an earlier run (e.g. glass): restore the imported material
            restore = unreal.load_asset(orig.split('.')[0])
            if restore:
                m.set_material(i, restore); changed = True
    if changed:
        eal.save_asset(p)
        unreal.log_warning('MASKED mesh ' + p.split('.')[0])
