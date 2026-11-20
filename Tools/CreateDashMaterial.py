# Mychron-style dash screen material (Milestone 8).
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateDashMaterial.py" -unattended -nosplash
#
# M_KartDash: unlit, emissive = Screen (the canvas render target the pawn draws the dash into) x Brightness.
# Brightness is in the scene's physical units (the sky / sun are physical, exposure EV ~12-14), so a sunlight-
# readable display needs hundreds to thousands (set at runtime by AKartVehiclePawn).
import unreal

mel = unreal.MaterialEditingLibrary
eal = unreal.EditorAssetLibrary
tools = unreal.AssetToolsHelpers.get_asset_tools()

full = '/Game/KartVisuals/M_KartDash'
if eal.does_asset_exist(full):
    eal.delete_asset(full)
mat = tools.create_asset('M_KartDash', '/Game/KartVisuals', unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property('shading_model', unreal.MaterialShadingModel.MSM_UNLIT)

tex = mel.create_material_expression(mat, unreal.MaterialExpressionTextureSampleParameter2D, -600, 0)
tex.set_editor_property('parameter_name', 'Screen')
tex.set_editor_property('texture', unreal.load_asset('/Engine/EngineResources/DefaultTexture'))
tex.set_editor_property('sampler_type', unreal.MaterialSamplerType.SAMPLERTYPE_COLOR)

bright = mel.create_material_expression(mat, unreal.MaterialExpressionScalarParameter, -600, 250)
bright.set_editor_property('parameter_name', 'Brightness')
bright.set_editor_property('default_value', 2000.0)

mul = mel.create_material_expression(mat, unreal.MaterialExpressionMultiply, -300, 100)
mel.connect_material_expressions(tex, 'RGB', mul, 'A')
mel.connect_material_expressions(bright, '', mul, 'B')
mel.connect_material_property(mul, '', unreal.MaterialProperty.MP_EMISSIVE_COLOR)

mel.recompile_material(mat)
eal.save_asset(full)
unreal.log_warning('CreateDashMaterial: saved ' + full)
