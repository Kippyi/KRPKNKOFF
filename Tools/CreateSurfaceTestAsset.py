# Creates /Game/KartVisuals/Test/SM_Pad_KartSurface_Gravel (a copy of the engine cube) used by
# -KartSurfaceSelfTest to verify the imported-art surface naming convention ("KartSurface_<Surface>"
# in the static mesh asset name).
#   UnrealEditor-Cmd.exe <project> -run=pythonscript -script="Tools/CreateSurfaceTestAsset.py" -unattended -nosplash
import unreal

DST = "/Game/KartVisuals/Test/SM_Pad_KartSurface_Gravel"
if unreal.EditorAssetLibrary.does_asset_exist(DST):
    unreal.EditorAssetLibrary.delete_asset(DST)
ok = unreal.EditorAssetLibrary.duplicate_asset("/Engine/BasicShapes/Cube", DST)
# The kart ground query traces COMPLEX collision (as real track meshes need); the engine cube only has a
# simple box, so make complex traces use it.
sm = unreal.load_asset(DST)
body = sm.get_editor_property("body_setup")
body.set_editor_property("collision_trace_flag", unreal.CollisionTraceFlag.CTF_USE_SIMPLE_AS_COMPLEX)
unreal.EditorAssetLibrary.save_asset(DST, only_if_is_dirty=False)
unreal.log(f"KARTVISUALS: test asset {DST} created={bool(ok)}")
