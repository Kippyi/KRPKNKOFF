# Imports the CC0 Poly Haven glTF models (Art/Models/PolyHaven/<id>/*.gltf) into /Game/Scenery/<id> and enables
# Nanite on dense meshes. Logs every static mesh (bounds in cm, triangle count) for the scenery placement code.
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/ImportScenery.py" -unattended -nosplash
import unreal, os, glob

SRC = os.path.join(unreal.Paths.project_dir(), 'Art', 'Models', 'PolyHaven')
eal = unreal.EditorAssetLibrary
tasks = []
for d in sorted(os.listdir(SRC)):
    for f in glob.glob(os.path.join(SRC, d, '*.gltf')):
        t = unreal.AssetImportTask()
        t.filename = f
        t.destination_path = f'/Game/Scenery/{d}'
        t.automated = True
        t.replace_existing = True
        t.save = True
        tasks.append(t)
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)

for p in sorted(eal.list_assets('/Game/Scenery', recursive=True)):
    a = unreal.load_asset(p)
    if not isinstance(a, unreal.StaticMesh):
        continue
    tris = a.get_num_triangles(0) if hasattr(a, 'get_num_triangles') else -1
    if tris > 20000:
        ns = a.get_editor_property('nanite_settings')
        if not ns.enabled:
            ns.enabled = True
            a.set_editor_property('nanite_settings', ns)
            eal.save_asset(p)
    b = a.get_bounding_box()
    unreal.log_warning('SCENERY %s tris %d min(%.0f %.0f %.0f) max(%.0f %.0f %.0f)' % (p.split('.')[0], tris, b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z))
