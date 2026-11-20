# Photographed sky: imports the CC0 Poly Haven pure-sky HDRIs (Art/Sky/*.exr, equirectangular, converted from .hdr
# by Blender) as 2D HDR textures and creates /Game/KartVisuals/M_KartSky: unlit "IsSky" material that maps the view
# direction to lat-long UVs itself (so the sun direction is known exactly), blends clear -> overcast, clamps the
# photographed sun disc (the directional light is the real sun) and scales to physical luminance.
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateSky.py" -unattended -nosplash
#
# Mapping (UE world, Z up): az = atan2(d.y, d.x), u = frac(az / 2pi + SkyYawOffset), v = 0.5 - asin(d.z) / pi.
# kloofendal_48d: sun at u = 0.595, elevation 47.9 deg -> with offset 0 the sun is at az 214.2 deg; the directional
# light (pointing FROM the sun) is then Rotation(pitch -47.9, yaw 34.2).
#   Params: Overcast (0..1), SkyIntensity (nits per HDR unit), ClearClamp (HDR units, sun disc clamp),
#           OvercastScale (relative brightness of the overcast photo), SkyYawOffset.
import unreal, os

eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()
SRC = os.path.join(unreal.Paths.project_dir(), 'Art', 'Sky')
DST = '/Game/KartVisuals/Sky'
names = {'kloofendal_48d_partly_cloudy_puresky': 'T_Sky_Clear', 'overcast_soil_puresky': 'T_Sky_Overcast'}
for src, name in names.items():
    t = unreal.AssetImportTask()
    t.filename = os.path.join(SRC, src + '.exr')
    t.destination_path = DST
    t.destination_name = name
    t.automated = True; t.replace_existing = True; t.save = False
    tools.import_asset_tasks([t])
    tex = unreal.load_asset(f'{DST}/{name}')
    tex.set_editor_property('compression_settings', unreal.TextureCompressionSettings.TC_HDR)
    tex.set_editor_property('mip_gen_settings', unreal.TextureMipGenSettings.TMGS_NO_MIPMAPS)
    tex.set_editor_property('address_x', unreal.TextureAddress.TA_WRAP)
    tex.set_editor_property('address_y', unreal.TextureAddress.TA_CLAMP)
    tex.set_editor_property('srgb', False)
    tex.set_editor_property('never_stream', True)
    tex.set_editor_property('lod_group', unreal.TextureGroup.TEXTUREGROUP_SKYBOX)
    eal.save_asset(f'{DST}/{name}')
    unreal.log_warning('SKY texture %s %s' % (name, type(tex).__name__))

full = '/Game/KartVisuals/M_KartSky'
if eal.does_asset_exist(full):
    eal.delete_asset(full)
mat = tools.create_asset('M_KartSky', '/Game/KartVisuals', unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property('shading_model', unreal.MaterialShadingModel.MSM_UNLIT)
mat.set_editor_property('two_sided', True)
mat.set_editor_property('is_sky', True)

def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)
def scalar(name, v, x, y):
    n = node(unreal.MaterialExpressionScalarParameter, x, y)
    n.set_editor_property('parameter_name', name); n.set_editor_property('default_value', v); return n
def custom(desc, code, out, ins, x, y):
    c = node(unreal.MaterialExpressionCustom, x, y)
    c.set_editor_property('output_type', out); c.set_editor_property('description', desc); c.set_editor_property('code', code)
    lst = []
    for k in ins:
        ci = unreal.CustomInput(); ci.set_editor_property('input_name', k); lst.append(ci)
    c.set_editor_property('inputs', lst)
    for k, v in ins.items():
        mel.connect_material_expressions(v, '', c, k)
    return c
O = unreal.CustomMaterialOutputType
cam = node(unreal.MaterialExpressionCameraVectorWS, -1400, 0)
yaw = scalar('SkyYawOffset', 0.0, -1400, 150)
uv = custom('SkyUV', '''
float3 d = -normalize(V);
float az = atan2(d.y, d.x);
float u = frac(az / 6.28318530718 + Yaw);
float v = 0.5 - asin(clamp(d.z, -1.0, 1.0)) / 3.14159265359;
// Pure-sky photos are black below the horizon: repeat the horizon row downward (seen past the terrain edge).
return float2(u, min(v, 0.4985));
''', O.CMOT_FLOAT2, {'V': cam, 'Yaw': yaw}, -1100, 0)
def tex(name, path, y):
    n = node(unreal.MaterialExpressionTextureSampleParameter2D, -800, y)
    n.set_editor_property('parameter_name', name)
    n.set_editor_property('texture', unreal.load_asset(path))
    n.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_LINEAR_COLOR)
    n.set_editor_property('mip_value_mode', unreal.TextureMipValueMode.TMVM_MIP_LEVEL)   # no derivatives: no seam at the u wrap
    mel.connect_material_expressions(uv, '', n, 'UVs')
    return n
clear = tex('ClearSky', f'{DST}/T_Sky_Clear', -200)
over = tex('OvercastSky', f'{DST}/T_Sky_Overcast', 200)
out = custom('SkyCombine', '''
float3 c = min(A.rgb, Clamp);
float3 o = B.rgb * OvercastScale;
return lerp(c, o, saturate(Overcast)) * Intensity;
''', O.CMOT_FLOAT3, {
    'A': clear, 'B': over,
    'Clamp': scalar('ClearClamp', 40.0, -500, 400),
    'OvercastScale': scalar('OvercastScale', 0.35, -500, 500),
    'Overcast': scalar('Overcast', 0.0, -500, 600),
    'Intensity': scalar('SkyIntensity', 6000.0, -500, 700),
}, -300, 0)
mel.connect_material_property(out, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR)
mel.recompile_material(mat)
eal.save_asset(full)
unreal.log_warning('SKY material created ' + full)
