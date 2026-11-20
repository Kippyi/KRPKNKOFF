# Nanite foliage fix: Poly Haven trees model every leaf as separate geometry (millions of tiny triangles). Without
# "Preserve Area", Nanite simplification erodes them away (trees render bare); the fallback mesh has none either.
# (StaticMeshEditorSubsystem is not available in commandlets: set the struct via set_editor_property + force save.)
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/FixSceneryFoliage.py" -unattended -nosplash
import unreal
eal = unreal.EditorAssetLibrary
for p in eal.list_assets('/Game/Scenery', recursive=True):
    if not any(k in p for k in ('jacaranda', 'tree_small', 'fir_sapling', 'shrub_')):
        continue
    m = unreal.load_asset(p)
    if not isinstance(m, unreal.StaticMesh):
        continue
    ns = m.get_editor_property('nanite_settings')
    ns.set_editor_property('enabled', True)
    ns.set_editor_property('shape_preservation', unreal.NaniteShapePreservation.PRESERVE_AREA)
    m.set_editor_property('nanite_settings', ns)
    ok = eal.save_loaded_asset(m, only_if_is_dirty=False)
    unreal.log_warning('FOLIAGE %s saved=%s' % (p.split('.')[0].split('/')[-1], ok))
