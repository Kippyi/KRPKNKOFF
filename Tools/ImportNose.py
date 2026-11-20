# Re-imports only the skinfront split by Tools/SplitNose.py (front panel + nose) into /Game/KartVisuals/BogKart.
#   UnrealEditor-Cmd.exe KRPKNKOFF.uproject -run=pythonscript -script="Tools/ImportNose.py" -unattended -nosplash
# Prints each mesh's bounds (cm) so the axis convention can be checked.
import unreal, os, glob

SRC = os.path.join(unreal.Paths.project_dir(), 'Art', 'BogKart')
DST = '/Game/KartVisuals/BogKart'
tasks = []
for f in [os.path.join(SRC, 'Body_skinfront.fbx'), os.path.join(SRC, 'Nose_skinfront.fbx')]:
    opts = unreal.FbxImportUI()
    opts.import_mesh = True
    opts.import_as_skeletal = False
    opts.import_materials = False
    opts.import_textures = False
    opts.import_animations = False
    opts.mesh_type_to_import = unreal.FBXImportType.FBXIT_STATIC_MESH
    sm = opts.static_mesh_import_data
    sm.combine_meshes = True
    sm.auto_generate_collision = False
    sm.generate_lightmap_u_vs = False
    sm.remove_degenerates = True
    sm.normal_import_method = unreal.FBXNormalImportMethod.FBXNIM_IMPORT_NORMALS
    sm.convert_scene = True
    sm.force_front_x_axis = False
    t = unreal.AssetImportTask()
    t.filename = f
    t.destination_path = DST
    t.destination_name = 'SM_Bog_' + os.path.splitext(os.path.basename(f))[0]
    t.automated = True
    t.replace_existing = True
    t.save = True
    t.options = opts
    tasks.append(t)
unreal.AssetToolsHelpers.get_asset_tools().import_asset_tasks(tasks)
for t in tasks:
    for p in t.imported_object_paths:
        m = unreal.load_asset(p)
        if isinstance(m, unreal.StaticMesh):
            b = m.get_bounding_box()
            unreal.log_warning('BOGKART %s min(%.1f %.1f %.1f) max(%.1f %.1f %.1f)' % (p, b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z))
