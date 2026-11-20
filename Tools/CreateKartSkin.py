# R2S livery skin: imports the user-supplied livery TGAs (project root) into /Game/KartVisuals/BogKart/Skin and creates
# /Game/KartVisuals/M_KartSkin (clear-coat paint whose base colour is a UV-mapped texture, parameter SkinTex).
# Run after Tools/SplitBogKart.py + Tools/ImportBogKart.py.
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateKartSkin.py" -unattended -nosplash
import unreal, os

eal = unreal.EditorAssetLibrary
mel = unreal.MaterialEditingLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()
ROOT = unreal.Paths.project_dir()
DST = '/Game/KartVisuals/BogKart/Skin'
TEX = {'R2S_KG509_FRONT.tga': 'T_Skin_Front', 'R2S_KG507_SIDEPOD.tga': 'T_Skin_Side', 'R2S_KG_REAR.tga': 'T_Skin_Rear',
       'sodi swheel.tga': 'T_Skin_SwGrip', 'm7 Base UV.tga': 'T_Skin_SwBase'}
for f, name in TEX.items():
    t = unreal.AssetImportTask()
    t.filename = os.path.join(ROOT, f)
    t.destination_path = DST
    t.destination_name = name
    t.automated = True; t.replace_existing = True; t.save = True
    tools.import_asset_tasks([t])
    unreal.log_warning('SKIN texture %s <- %s (%s)' % (name, f, 'ok' if eal.does_asset_exist(f'{DST}/{name}') else 'MISSING'))

full = '/Game/KartVisuals/M_KartSkin'
if eal.does_asset_exist(full):
    eal.delete_asset(full)
mat = tools.create_asset('M_KartSkin', '/Game/KartVisuals', unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property('shading_model', unreal.MaterialShadingModel.MSM_CLEAR_COAT)   # coat / coat roughness: engine defaults
mat.set_editor_property('two_sided', True)
tex = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSampleParameter2D, -600, 0)
tex.set_editor_property('parameter_name', 'SkinTex')
tex.set_editor_property('texture', unreal.load_asset(f'{DST}/T_Skin_Front'))
mel.connect_material_property(tex, 'RGB', unreal.MaterialProperty.MP_BASE_COLOR)
for prop, name, v, y in [(unreal.MaterialProperty.MP_ROUGHNESS, 'Roughness', 0.3, 300), (unreal.MaterialProperty.MP_METALLIC, 'Metallic', 0.0, 400)]:
    n = mel.create_material_expression(mat, unreal.MaterialExpressionScalarParameter, -400, y)
    n.set_editor_property('parameter_name', name); n.set_editor_property('default_value', v)
    mel.connect_material_property(n, '', prop)
mel.recompile_material(mat)
eal.save_asset(full)
unreal.log_warning('SKIN material created ' + full)
