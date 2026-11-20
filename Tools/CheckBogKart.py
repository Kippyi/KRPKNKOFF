# Logs the bounds (cm) of every static mesh in /Game/KartVisuals/BogKart (axis-convention check).
import unreal
for p in sorted(unreal.EditorAssetLibrary.list_assets('/Game/KartVisuals/BogKart', recursive=True)):
    m = unreal.load_asset(p)
    if isinstance(m, unreal.StaticMesh):
        b = m.get_bounding_box()
        unreal.log_warning('BOGKART %-48s min(%7.1f %7.1f %7.1f) max(%7.1f %7.1f %7.1f) slots %d' % (p.split('/')[-1], b.min.x, b.min.y, b.min.z, b.max.x, b.max.y, b.max.z, len(m.static_materials)))
    else:
        unreal.log_warning('BOGKART other %s %s' % (p, type(m).__name__))
