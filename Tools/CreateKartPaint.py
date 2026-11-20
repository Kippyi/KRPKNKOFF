# Creates /Game/KartVisuals/M_KartPaint: clear-coat car paint with a livery painted in OBJECT space (no UVs or
# textures needed; works on imported bodywork whose source textures are missing).
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/CreateKartPaint.py" -unattended -nosplash
#
# Object space of the bodywork parts (Tools/SplitBogKart.py): X forward of the rear axle, Y right, Z up from the
# axle centre, cm. Livery zones (all runtime parameters, set per kart by AKartVehiclePawn):
#   PrimaryColor     base paint (the kart's colour)
#   SecondaryColor   lower band below LowZ (cm)
#   AccentColor      diagonal swoosh  |z - (x*SwooshSlope + SwooshOffset)| < SwooshWidth, plus a pinstripe on the band edge
#   Roughness / Metallic (base coat); lacquer = engine clear-coat defaults (coat 1.0, roughness 0.1)
import unreal

PATH, NAME = "/Game/KartVisuals", "M_KartPaint"
mel = unreal.MaterialEditingLibrary
full = f"{PATH}/{NAME}"
if unreal.EditorAssetLibrary.does_asset_exist(full):
    unreal.EditorAssetLibrary.delete_asset(full)
mat = unreal.AssetToolsHelpers.get_asset_tools().create_asset(NAME, PATH, unreal.Material, unreal.MaterialFactoryNew())
mat.set_editor_property("shading_model", unreal.MaterialShadingModel.MSM_CLEAR_COAT)
mat.set_editor_property("two_sided", True)

def node(cls, x, y):
    return mel.create_material_expression(mat, cls, x, y)
def scalar(name, v, x, y):
    n = node(unreal.MaterialExpressionScalarParameter, x, y)
    n.set_editor_property("parameter_name", name); n.set_editor_property("default_value", v); return n
def vector(name, v, x, y):
    n = node(unreal.MaterialExpressionVectorParameter, x, y)
    n.set_editor_property("parameter_name", name); n.set_editor_property("default_value", unreal.LinearColor(*v)); return n

wp = node(unreal.MaterialExpressionWorldPosition, -1400, 0)
tp = node(unreal.MaterialExpressionTransformPosition, -1200, 0)
tp.set_editor_property("transform_source_type", unreal.MaterialPositionTransformSource.TRANSFORMPOSSOURCE_WORLD)
tp.set_editor_property("transform_type", unreal.MaterialPositionTransformSource.TRANSFORMPOSSOURCE_LOCAL)
mel.connect_material_expressions(wp, "", tp, "")

params = {
    "Prim": vector("PrimaryColor", (0.72, 0.04, 0.03, 1), -1200, 200),
    "Sec": vector("SecondaryColor", (0.85, 0.86, 0.88, 1), -1200, 400),
    "Acc": vector("AccentColor", (0.02, 0.02, 0.025, 1), -1200, 600),
    "LowZ": scalar("LowZ", -1.0, -1200, 800),
    "Slope": scalar("SwooshSlope", -0.10, -1200, 900),
    "Off": scalar("SwooshOffset", 16.0, -1200, 1000),
    "W": scalar("SwooshWidth", 2.2, -1200, 1100),
}
custom = node(unreal.MaterialExpressionCustom, -800, 300)
custom.set_editor_property("output_type", unreal.CustomMaterialOutputType.CMOT_FLOAT3)
custom.set_editor_property("description", "KartLivery")
custom.set_editor_property("code", """
float x = P.x, z = P.z;
float3 c = Prim;
float low = 1.0 - smoothstep(LowZ - 0.4, LowZ + 0.4, z);
c = lerp(c, Sec, low);
float pin = 1.0 - smoothstep(0.25, 0.55, abs(z - (LowZ + 1.3)));
c = lerp(c, Acc, pin);
float d = abs(z - (x * Slope + Off));
float sw = 1.0 - smoothstep(W - 0.3, W + 0.3, d);
c = lerp(c, Acc, sw);
float edge = (1.0 - smoothstep(0.15, 0.4, abs(d - W - 0.9))) * (1.0 - low);
c = lerp(c, Sec, edge);
return c;
""")
inputs = []
for n in ["P"] + list(params.keys()):
    ci = unreal.CustomInput(); ci.set_editor_property("input_name", n); inputs.append(ci)
custom.set_editor_property("inputs", inputs)
mel.connect_material_expressions(tp, "", custom, "P")
for k, v in params.items():
    mel.connect_material_expressions(v, "", custom, k)
mel.connect_material_property(custom, "", unreal.MaterialProperty.MP_BASE_COLOR)

for prop, name, v, y in [(unreal.MaterialProperty.MP_ROUGHNESS, "Roughness", 0.32, 1300),
                         (unreal.MaterialProperty.MP_METALLIC, "Metallic", 0.05, 1400),
                         ]:
    # ClearCoat / ClearCoatRoughness (CustomData0/1) are not reachable from Python: left unconnected they use the
    # engine defaults 1.0 / 0.1 (a full lacquer coat).
    mel.connect_material_property(scalar(name, v, -400, y), "", prop)

mel.recompile_material(mat)
unreal.EditorAssetLibrary.save_asset(full)
unreal.log_warning("KARTPAINT created " + full)
