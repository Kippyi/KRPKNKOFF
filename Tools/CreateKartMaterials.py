# Creates the master material /Game/KartVisuals/M_KartSurface used by the procedural visuals.
# Run headless (does not need the interactive editor):
#   UnrealEditor-Cmd.exe <project> -run=pythonscript -script="Tools/CreateKartMaterials.py" -unattended -nosplash
#
# Parameters (set per material instance at runtime):
#   ColorA, ColorB (vector)       base colour range, blended by world-space noise
#   MacroScale, MicroScale        noise frequencies (per cm of world position)
#   MacroAmount                   weight of the large-scale patches vs the fine grain
#   Roughness, RoughnessVar       base roughness and its noise-driven variation
#   Metallic                      metallic value
#   UseVertexColor (0/1)          multiply base colour by mesh vertex colour (kerb stripes, tyre walls)
import unreal

PATH = "/Game/KartVisuals"
NAME = "M_KartSurface"
mel = unreal.MaterialEditingLibrary
assets = unreal.AssetToolsHelpers.get_asset_tools()

full = f"{PATH}/{NAME}"
if unreal.EditorAssetLibrary.does_asset_exist(full):
    unreal.EditorAssetLibrary.delete_asset(full)
mat = assets.create_asset(NAME, PATH, unreal.Material, unreal.MaterialFactoryNew())

def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)

def scalar(name, default, x, y):
    n = node(unreal.MaterialExpressionScalarParameter, x, y)
    n.set_editor_property("parameter_name", name)
    n.set_editor_property("default_value", default)
    return n

def vector(name, default, x, y):
    n = node(unreal.MaterialExpressionVectorParameter, x, y)
    n.set_editor_property("parameter_name", name)
    n.set_editor_property("default_value", default)
    return n

def connect(a, a_out, b, b_in):
    mel.connect_material_expressions(a, a_out, b, b_in)

world = node(unreal.MaterialExpressionWorldPosition, -1400, 0)

# Large-scale patches
macro_scale = scalar("MacroScale", 0.004, -1400, 150)
macro_pos = node(unreal.MaterialExpressionMultiply, -1200, 50)
connect(world, "", macro_pos, "A"); connect(macro_scale, "", macro_pos, "B")
macro = node(unreal.MaterialExpressionNoise, -1000, 50)
macro.set_editor_property("scale", 1.0)
macro.set_editor_property("levels", 4)
macro.set_editor_property("output_min", 0.0)
macro.set_editor_property("output_max", 1.0)
connect(macro_pos, "", macro, "Position")

# Fine grain (aggregate)
micro_scale = scalar("MicroScale", 0.35, -1400, 300)
micro_pos = node(unreal.MaterialExpressionMultiply, -1200, 250)
connect(world, "", micro_pos, "A"); connect(micro_scale, "", micro_pos, "B")
micro = node(unreal.MaterialExpressionNoise, -1000, 250)
micro.set_editor_property("scale", 1.0)
micro.set_editor_property("levels", 2)
micro.set_editor_property("output_min", 0.0)
micro.set_editor_property("output_max", 1.0)
connect(micro_pos, "", micro, "Position")

# Blend: n = lerp(micro, macro, MacroAmount)
macro_amount = scalar("MacroAmount", 0.5, -1000, 450)
mix = node(unreal.MaterialExpressionLinearInterpolate, -750, 150)
connect(micro, "", mix, "A"); connect(macro, "", mix, "B"); connect(macro_amount, "", mix, "Alpha")

# Base colour = lerp(ColorA, ColorB, n) * (UseVertexColor ? VertexColor : 1)
col_a = vector("ColorA", unreal.LinearColor(0.04, 0.04, 0.045, 1.0), -750, -200)
col_b = vector("ColorB", unreal.LinearColor(0.09, 0.09, 0.095, 1.0), -750, -50)
base = node(unreal.MaterialExpressionLinearInterpolate, -500, -100)
connect(col_a, "", base, "A"); connect(col_b, "", base, "B"); connect(mix, "", base, "Alpha")
vcol = node(unreal.MaterialExpressionVertexColor, -500, 100)
use_vc = scalar("UseVertexColor", 0.0, -500, 250)
one = node(unreal.MaterialExpressionConstant3Vector, -500, 350)
one.set_editor_property("constant", unreal.LinearColor(1, 1, 1, 1))
vc_sel = node(unreal.MaterialExpressionLinearInterpolate, -300, 200)
connect(one, "", vc_sel, "A"); connect(vcol, "", vc_sel, "B"); connect(use_vc, "", vc_sel, "Alpha")
final_col = node(unreal.MaterialExpressionMultiply, -150, 0)
connect(base, "", final_col, "A"); connect(vc_sel, "", final_col, "B")
mel.connect_material_property(final_col, "", unreal.MaterialProperty.MP_BASE_COLOR)

# Roughness = Roughness + (n - 0.5) * RoughnessVar
rough = scalar("Roughness", 0.85, -500, 500)
rough_var = scalar("RoughnessVar", 0.15, -500, 600)
centred = node(unreal.MaterialExpressionSubtract, -500, 700)
connect(mix, "", centred, "A")
centred.set_editor_property("const_b", 0.5)
var_term = node(unreal.MaterialExpressionMultiply, -300, 650)
connect(centred, "", var_term, "A"); connect(rough_var, "", var_term, "B")
rough_sum = node(unreal.MaterialExpressionAdd, -150, 550)
connect(rough, "", rough_sum, "A"); connect(var_term, "", rough_sum, "B")
mel.connect_material_property(rough_sum, "", unreal.MaterialProperty.MP_ROUGHNESS)

metal = scalar("Metallic", 0.0, -150, 750)
mel.connect_material_property(metal, "", unreal.MaterialProperty.MP_METALLIC)

# Two-sided: procedural kart/track geometry is seen from all sides (thin panels, tubes).
mat.set_editor_property("two_sided", True)
# Usage flags: a cooked game cannot compile missing variants on demand (the editor can), so every
# component type that uses this material must be flagged. Tyre barriers are instanced static meshes.
mat.set_editor_property("used_with_instanced_static_meshes", True)
mel.recompile_material(mat)
unreal.EditorAssetLibrary.save_asset(full, only_if_is_dirty=False)
unreal.log(f"KARTVISUALS: created {full}")
