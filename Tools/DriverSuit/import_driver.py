# Imports Art/Driver/*.fbx (Tools/DriverSuit/make_driver.py) into /Game/KartVisuals/Driver. The helmet keeps its own
# materials / textures (Arai GP7 TGAs in the project root); the suit parts get runtime material classes.
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/DriverSuit/import_driver.py" -unattended -nosplash
import unreal, os, glob
SRC = os.path.join(unreal.Paths.project_dir(), 'Art', 'Driver')
for f in sorted(glob.glob(os.path.join(SRC, '*.fbx'))):
    helmet = 'Helmet' in f
    opts = unreal.FbxImportUI()
    opts.import_mesh = True; opts.import_as_skeletal = False; opts.import_animations = False
    opts.import_materials = helmet; opts.import_textures = helmet
    opts.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
    sm = opts.static_mesh_import_data
    sm.combine_meshes = True; sm.auto_generate_collision = False; sm.generate_lightmap_u_vs = False
    sm.normal_import_method = unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
    t = unreal.AssetImportTask()
    t.filename = f; t.destination_path = '/Game/KartVisuals/Driver'
    t.destination_name = 'SM_' + os.path.splitext(os.path.basename(f))[0]
    t.automated = True; t.replace_existing = True; t.save = True; t.options = opts
    unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks([t])
for p in sorted(unreal.EditorAssetLibrary.list_assets('/Game/KartVisuals/Driver', recursive=True)):
    a = unreal.load_asset(p)
    if isinstance(a, unreal.StaticMesh):
        b = a.get_bounding_box()
        unreal.log_warning('DRIVER %s slots %d min(%.0f %.0f %.0f) max(%.0f %.0f %.0f)' % (p.split('.')[0].split('/')[-1], len(a.static_materials), b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z))
    elif isinstance(a, unreal.MaterialInterface) or isinstance(a, unreal.Texture):
        unreal.log_warning('DRIVER asset %s %s' % (p.split('.')[0].split('/')[-1], type(a).__name__))

# --- M_DriverSuit: the suit's own Tripo PBR maps (racing_suit_3d_model_*.jpg/png, project root; UVs kept by make_driver).
# Base colour (cleaned) + roughness; the normal map is imported but not used. The metallic map is NOT used: it averages 0.37, which is non-physical for Nomex fabric
# and leather (dielectrics), so metallic = 0. Roughness is remapped 0.55..1.0 (the raw map, 0.37-0.67, reads as satin; a
# fabric suit is matte - ASSUMPTION, visual). Human drivers get this; AI drivers keep the flat 'suit' colour class.
eal = unreal.EditorAssetLibrary; mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()
ROOT = unreal.Paths.project_dir(); DST = '/Game/KartVisuals/Driver'
# base colour: the cleaned flat-panel version (Tools/DriverSuit/clean_suit_texture.py -> Art/Driver), not the raw smeared map
TEX = {os.path.join('Art', 'Driver', 'suit_basecolor_clean.png'): ('T_Suit_BaseColor', None),
       'racing_suit_3d_model_normal.jpg': ('T_Suit_Normal', unreal.TextureCompressionSettings.TC_NORMALMAP),
       'racing_suit_3d_model_roughness.png': ('T_Suit_Roughness', unreal.TextureCompressionSettings.TC_GRAYSCALE)}
for f, (name, comp) in TEX.items():
    t = unreal.AssetImportTask(); t.filename = os.path.join(ROOT, f); t.destination_path = DST; t.destination_name = name
    t.automated = True; t.replace_existing = True; t.save = True
    tools.import_asset_tasks([t])
    tex = unreal.load_asset(f'{DST}/{name}')
    if comp is not None:
        tex.set_editor_property('compression_settings', comp); tex.set_editor_property('srgb', False)
        eal.save_asset(f'{DST}/{name}')
    unreal.log_warning('DRIVER texture %s %s' % (name, 'ok' if tex else 'MISSING'))
full = f'{DST}/M_DriverSuit'
if eal.does_asset_exist(full): eal.delete_asset(full)
mat = tools.create_asset('M_DriverSuit', DST, unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property('two_sided', True)   # open neck / wrist cuts
bc = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSampleParameter2D, -700, -200)
bc.set_editor_property('parameter_name', 'BaseTex'); bc.set_editor_property('texture', unreal.load_asset(f'{DST}/T_Suit_BaseColor'))
mel.connect_material_property(bc, 'RGB', unreal.MaterialProperty.MP_BASE_COLOR)
nm = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -700, 100)
nm.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_NORMAL)
nm.set_editor_property('texture', unreal.load_asset(f'{DST}/T_Suit_Normal'))
# NOT connected: the AI normal map is smeared like the base colour and shows as dark mottling in game; the mesh has
# real folds. (Node kept for reference.)
rg = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSample, -900, 400)
rg.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_GRAYSCALE)
rg.set_editor_property('texture', unreal.load_asset(f'{DST}/T_Suit_Roughness'))
lp = mel.create_material_expression(mat, unreal.MaterialExpressionLinearInterpolate, -400, 400)
lp.set_editor_property('const_a', 0.55); lp.set_editor_property('const_b', 1.0)
mel.connect_material_expressions(rg, 'R', lp, 'Alpha')
mel.connect_material_property(lp, '', unreal.MaterialProperty.MP_ROUGHNESS)
mt = mel.create_material_expression(mat, unreal.MaterialExpressionConstant, -400, 600)
mel.connect_material_property(mt, '', unreal.MaterialProperty.MP_METALLIC)
mel.recompile_material(mat); eal.save_asset(full)
unreal.log_warning('DRIVER material created ' + full)
