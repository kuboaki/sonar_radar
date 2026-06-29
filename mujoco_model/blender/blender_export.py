"""
Blender用スクリプト: STL形式で直接書き出し（numpy-stl使用）

対応モデル: sonar_radar09.io / sonar_radar09.blend
  Blenderシーン上のオブジェクト階層:
    radar_base      … 台座グレーパーツ（地面プレート含む、マーカー/壁を含まない）
    marker_red      … 赤マーカーブロック
    marker_blue     … 青マーカーブロック
    radar_dome      … ドーム（36Tギアを含まない、センサーを含まない）
    bevel_gear_12   … モーター側12Tベベルギア（旋回軸=このEMPTY原点）
    bevel_gear_36   … ドーム側36Tベベルギア一式（旋回軸=このEMPTY原点）
    obstacle_wall_a … 壁ブロックA
    obstacle_wall_b … 壁ブロックB
    starter         … スターターユニット（フォースセンサー+press_block）
    37316c01.dat    … 距離センサー（ドーム内）
    37308c01.dat    … カラーセンサー（ドーム内）

変更点（sonar_radar08→09）:
  - starter サブモデルを新規追加
  - センサーSTL出力を廃止（libspikehat_sim コンポーネントを使用）
  - センサーの <include> コンポーネント用マウント pos を出力

アプローチ:
  - matrix_world で頂点をワールド座標に変換
  - オフセット（XY中心=0, Z底面=0 or XYZ中心=0）を適用
  - Z軸回転を適用（LDraw空間内）
  - LDraw → MuJoCo 座標変換 + SCALE適用:
      mj_x = -rx * SCALE
      mj_y = -ry * SCALE
      mj_z =  rz * SCALE

出力 STL:
  radar_base_gray.stl, radar_base_red.stl, radar_base_blue.stl
  bevel_gear_12.stl, bevel_gear_36.stl
  radar_dome.stl
  obstacle_wall_a.stl, obstacle_wall_b.stl

センサー/starterはSTLを出力しない（libspikehat_sim の meshes/ を使用）。
コンポーネント用マウント pos のみをログに出力する。

使い方:
  Blenderのスクリプトエディタで sonar_radar09.blend を開き「スクリプトを実行」
"""

import bpy
import mathutils
import math
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Projects/sonar_radar/mujoco_model/meshes"
LOG_PATH   = "/Users/kuboaki/Projects/sonar_radar/mujoco_model/blender/blender_export_log.txt"
SCALE = 0.0004  # LDU → m


# ── ログ ─────────────────────────────────────────────────

class _Tee:
    def __init__(self, filepath):
        self._stdout = sys.stdout
        self._file   = open(filepath, "w", encoding="utf-8")
    def write(self, text):
        self._stdout.write(text)
        self._file.write(text)
    def flush(self):
        self._stdout.flush()
        self._file.flush()
    def close(self):
        sys.stdout = self._stdout
        self._file.close()

_tee = _Tee(LOG_PATH)
sys.stdout = _tee
print("# blender_export_log")
print(f"# LOG_PATH: {LOG_PATH}")


# ── メッシュ収集 ──────────────────────────────────────────

def collect_mesh_descendants(root, stop_at_empty=False):
    result = []
    for child in root.children:
        if child.type == 'MESH':
            result.append(child)
            result.extend(collect_mesh_descendants(child, stop_at_empty))
        elif child.type == 'EMPTY':
            if stop_at_empty:
                print(f"    SKIP EMPTY: {child.name}")
            else:
                result.extend(collect_mesh_descendants(child, stop_at_empty))
    return result


# ── バウンディングボックス ────────────────────────────────

def combined_bbox(meshes):
    xs, ys, zs = [], [], []
    for obj in meshes:
        mat = obj.matrix_world
        for v in obj.data.vertices:
            wv = mat @ v.co
            xs.append(wv.x); ys.append(wv.y); zs.append(wv.z)
    return (min(xs), max(xs)), (min(ys), max(ys)), (min(zs), max(zs))


# ── STL エクスポート ──────────────────────────────────────

def export_stl(meshes, name, center_mode="bottom_z", rotate_z_deg=0.0,
               rotor_axis_obj=None, out_filename="out.stl",
               shared_offset=None):
    """
    center_mode  : 'bottom_z'   = XY中心+Z底面を0
                   'center'     = XYZ中心を0
                   'rotor_axis' = rotor_axis_obj の位置を原点に（旋回軸中心）
                   'shared'     = shared_offset(dx,dy,dz)をそのまま使用
    shared_offset: (dx, dy, dz) center_mode='shared' のときに使うオフセット
    Returns half_Z [m] or None
    """
    if not meshes:
        print("  ERROR: メッシュリストが空です")
        return None

    print(f"  MESHオブジェクト数: {len(meshes)}")

    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(meshes)
    cx, cy, cz = (x0+x1)/2, (y0+y1)/2, (z0+z1)/2
    print(f"  統合bbox (変換前):")
    print(f"    X[{x0:.1f}, {x1:.1f}]  Y[{y0:.1f}, {y1:.1f}]  Z[{z0:.1f}, {z1:.1f}] LDU")

    if center_mode == "shared" and shared_offset is not None:
        dx, dy, dz = shared_offset
        print(f"  共有オフセット使用: ({dx:.2f}, {dy:.2f}, {dz:.2f}) LDU")
    elif center_mode == "bottom_z":
        dx, dy, dz = -cx, -cy, -z0
    elif center_mode == "rotor_axis":
        if rotor_axis_obj is None:
            print("  WARNING: rotor_axis_obj が未指定。'center' にフォールバック")
            dx, dy, dz = -cx, -cy, -cz
        else:
            ax = rotor_axis_obj.matrix_world.translation
            dx, dy, dz = -ax.x, -ax.y, -ax.z
            print(f"  rotor_axis_obj: {rotor_axis_obj.name}")
            print(f"  旋回軸 (Blender世界座標): ({ax.x:.2f}, {ax.y:.2f}, {ax.z:.2f}) LDU")
    else:
        dx, dy, dz = -cx, -cy, -cz
    print(f"  オフセット: ({dx:.2f}, {dy:.2f}, {dz:.2f}) LDU")

    angle = math.radians(rotate_z_deg)
    cos_a = math.cos(angle)
    sin_a = math.sin(angle)

    all_verts = []
    triangles = []
    v_offset  = 0

    depsgraph = bpy.context.evaluated_depsgraph_get()

    for obj in meshes:
        eval_obj = obj.evaluated_get(depsgraph)
        mesh     = eval_obj.to_mesh()
        mat      = obj.matrix_world
        wv_list  = [mat @ v.co for v in mesh.vertices]

        for v in wv_list:
            x, y, z = v.x + dx, v.y + dy, v.z + dz
            rx = cos_a * x - sin_a * y
            ry = sin_a * x + cos_a * y
            rz = z
            all_verts.append([-rx * SCALE, -ry * SCALE, rz * SCALE])

        for poly in mesh.polygons:
            vlist = list(poly.vertices)
            for i in range(1, len(vlist) - 1):
                triangles.append([v_offset + vlist[0],
                                   v_offset + vlist[i],
                                   v_offset + vlist[i + 1]])
        v_offset += len(wv_list)
        eval_obj.to_mesh_clear()

    stl_data = stl_mesh.Mesh(np.zeros(len(triangles), dtype=stl_mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j, vi in enumerate(tri):
            stl_data.vectors[i][j] = all_verts[vi]

    filepath = os.path.join(OUTPUT_DIR, out_filename)
    stl_data.save(filepath)
    print(f"  Saved: {filepath}")
    print(f"  頂点数: {len(all_verts)}  三角面数: {len(triangles)}")

    half_Z    = (y1 - y0) / 2 * SCALE
    mj_z_min  = -(y1 + dy) * SCALE
    mj_z_max  = -(y0 + dy) * SCALE
    print(f"  MuJoCo Z底面: {mj_z_min:.4f} m  Z上面: {mj_z_max:.4f} m")
    print(f"  half_Z: {half_Z:.4f} m")
    if rotate_z_deg != 0.0:
        print(f"  Z軸回転: {rotate_z_deg}°")

    return half_Z


# ── シーン確認 ────────────────────────────────────────────

print("\n" + "=" * 60)
print("シーン内オブジェクト一覧")
print("=" * 60)
for o in sorted(bpy.data.objects, key=lambda x: x.name):
    pname = o.parent.name if o.parent else "(root)"
    extra = f"  verts={len(o.data.vertices)}" if o.type == 'MESH' else ""
    print(f"  [{o.type:5s}] {o.name:40s} parent={pname}{extra}")


# ── メッシュ収集 ──────────────────────────────────────────

base_root    = bpy.data.objects.get("radar_base")
panel_root   = bpy.data.objects.get("ground_plate")
red_root     = bpy.data.objects.get("marker_red")
blue_root    = bpy.data.objects.get("marker_blue")
dome_root    = bpy.data.objects.get("radar_dome")
gear12_root  = bpy.data.objects.get("bevel_gear_12")
gear36_root  = bpy.data.objects.get("bevel_gear_36")
wall_a_root  = bpy.data.objects.get("obstacle_wall_a")
wall_b_root  = bpy.data.objects.get("obstacle_wall_b")
starter_root    = bpy.data.objects.get("starter")
press_block_obj = bpy.data.objects.get("press_block copy") or bpy.data.objects.get("press_block")

base_meshes   = collect_mesh_descendants(base_root,   stop_at_empty=True) if base_root   else []
panel_meshes  = collect_mesh_descendants(panel_root)  if panel_root  else []
red_meshes    = collect_mesh_descendants(red_root)    if red_root    else []
blue_meshes   = collect_mesh_descendants(blue_root)   if blue_root   else []
dome_meshes   = collect_mesh_descendants(dome_root,   stop_at_empty=True) if dome_root   else []
gear12_meshes = collect_mesh_descendants(gear12_root) if gear12_root else []
gear36_meshes = collect_mesh_descendants(gear36_root) if gear36_root else []
wall_a_meshes = collect_mesh_descendants(wall_a_root) if wall_a_root else []
wall_b_meshes = collect_mesh_descendants(wall_b_root) if wall_b_root else []

print(f"\n収集結果:")
print(f"  radar_base      = {len(base_meshes)} MESH  {'OK' if base_root   else 'NOT FOUND'}")
print(f"  ground_plate    = {len(panel_meshes)} MESH  {'OK' if panel_root  else 'NOT FOUND'}")
print(f"  marker_red      = {len(red_meshes)} MESH  {'OK' if red_root    else 'NOT FOUND'}")
print(f"  marker_blue     = {len(blue_meshes)} MESH  {'OK' if blue_root   else 'NOT FOUND'}")
print(f"  radar_dome      = {len(dome_meshes)} MESH  {'OK' if dome_root   else 'NOT FOUND'}")
print(f"  bevel_gear_12   = {len(gear12_meshes)} MESH  {'OK' if gear12_root else 'NOT FOUND'}")
print(f"  bevel_gear_36   = {len(gear36_meshes)} MESH  {'OK' if gear36_root else 'NOT FOUND'}")
print(f"  obstacle_wall_a = {len(wall_a_meshes)} MESH  {'OK' if wall_a_root else 'NOT FOUND'}")
print(f"  obstacle_wall_b = {len(wall_b_meshes)} MESH  {'OK' if wall_b_root else 'NOT FOUND'}")


# ── STL面数プリフライトチェック ────────────────────────────

def count_triangles(meshes):
    return sum(len(p.vertices) - 2 for o in meshes for p in o.data.polygons)

LIMIT = 200000
_PANEL_NAME = "39369.dat"
_panel_plate = [o for o in panel_meshes if o.name == _PANEL_NAME]
_panel_other = [o for o in panel_meshes if o.name != _PANEL_NAME]
_base_gray   = base_meshes + _panel_other

tri_gray  = count_triangles(_base_gray)
tri_plate = count_triangles(_panel_plate)
tri_dome  = count_triangles(dome_meshes)

print(f"\n{'=' * 60}")
print(f"STL面数プリフライトチェック (上限 {LIMIT:,})")
print(f"{'=' * 60}")
print(f"  radar_base_gray ({len(_base_gray)}パーツ):  {tri_gray:>7,}  {'OK' if tri_gray <= LIMIT else 'OVER!'}")
print(f"  radar_base_panel ({_PANEL_NAME}のみ): {tri_plate:>7,}  {'OK' if tri_plate <= LIMIT else 'OVER!'}")
print(f"  radar_dome:                    {tri_dome:>7,}  {'OK' if tri_dome <= LIMIT else 'OVER!'}")


# ── radar_base 共有オフセット計算 ─────────────────────────

print("\n" + "=" * 60)
print("radar_base 共有オフセット計算")
print("=" * 60)

# 共有オフセットは ground_plate も含めた全体の bbox から計算する。
# こうすることで ground_plate を分離しても座標原点が変わらない。
all_base_meshes = base_meshes + panel_meshes + red_meshes + blue_meshes
if all_base_meshes:
    (x0,x1),(y0,y1),(z0,z1) = combined_bbox(all_base_meshes)
    cx, cy = (x0+x1)/2, (y0+y1)/2
    base_shared_offset = (-cx, -cy, -z0)
    print(f"  全baseパーツ統合bbox:")
    print(f"    X[{x0:.1f}, {x1:.1f}]  Y[{y0:.1f}, {y1:.1f}]  Z[{z0:.1f}, {z1:.1f}] LDU")
    print(f"  共有オフセット: ({base_shared_offset[0]:.2f}, {base_shared_offset[1]:.2f}, {base_shared_offset[2]:.2f}) LDU")
else:
    base_shared_offset = None
    print("  WARNING: baseパーツが見つかりません")


# ground_plate 内の 39369.dat（パネルプレート）は頂点数が多いため単独STLに分離。
# それ以外の ground_plate パーツは radar_base_gray に統合して面数上限内に収める。
PANEL_PLATE_NAME   = "39369.dat"
panel_plate_meshes = [o for o in panel_meshes if o.name == PANEL_PLATE_NAME]
panel_other_meshes = [o for o in panel_meshes if o.name != PANEL_PLATE_NAME]
base_gray_meshes   = base_meshes + panel_other_meshes

print("\n" + "=" * 60)
print(f"radar_base_panel 処理（{PANEL_PLATE_NAME} のみ）")
print("=" * 60)

export_stl(
    meshes        = panel_plate_meshes,
    name          = "radar_base_panel",
    center_mode   = "shared",
    shared_offset = base_shared_offset,
    out_filename  = "radar_base_panel.stl",
)

print("\n" + "=" * 60)
print(f"radar_base_gray 処理（radar_base + ground_plate の {PANEL_PLATE_NAME} 以外）")
print("=" * 60)

base_half_Z = export_stl(
    meshes        = base_gray_meshes,
    name          = "radar_base_gray",
    center_mode   = "shared",
    shared_offset = base_shared_offset,
    out_filename  = "radar_base_gray.stl",
)

# ── marker_red エクスポート ───────────────────────────────

print("\n" + "=" * 60)
print("marker_red 処理")
print("=" * 60)

export_stl(
    meshes        = red_meshes,
    name          = "marker_red",
    center_mode   = "shared",
    shared_offset = base_shared_offset,
    out_filename  = "radar_base_red.stl",
)

# ── marker_blue エクスポート ──────────────────────────────

print("\n" + "=" * 60)
print("marker_blue 処理")
print("=" * 60)

export_stl(
    meshes        = blue_meshes,
    name          = "marker_blue",
    center_mode   = "shared",
    shared_offset = base_shared_offset,
    out_filename  = "radar_base_blue.stl",
)

# ── bevel_gear_12 エクスポート ────────────────────────────

print("\n" + "=" * 60)
print("bevel_gear_12 処理")
print("=" * 60)

export_stl(
    meshes         = gear12_meshes,
    name           = "bevel_gear_12",
    center_mode    = "rotor_axis" if gear12_root else "center",
    rotor_axis_obj = gear12_root,
    out_filename   = "bevel_gear_12.stl",
)

# ── bevel_gear_36 エクスポート ────────────────────────────

print("\n" + "=" * 60)
print("bevel_gear_36 処理")
print("=" * 60)

export_stl(
    meshes         = gear36_meshes,
    name           = "bevel_gear_36",
    center_mode    = "rotor_axis" if gear36_root else "center",
    rotor_axis_obj = gear36_root,
    out_filename   = "bevel_gear_36.stl",
)

# ── センサーsite位置計算 ──────────────────────────────────

PLATE_H    = 8  * SCALE   # 1プレート高さ = 8 LDU = 3.2mm = 0.0032m
STUD_PITCH = 20 * SCALE   # 水平方向1スタッドピッチ = 20 LDU = 8mm = 0.008m


def compute_local_body_pos(obj, rotor_axis_obj):
    """
    obj の原点位置を rotor_axis_obj 中心の dome body ローカル座標（MJCF pos、単位 m）に変換する。
    センサーを独立 body として配置するときの pos 値を求める。
    """
    ax  = rotor_axis_obj.matrix_world.translation
    pos = obj.matrix_world.translation
    x, y, z = pos.x - ax.x, pos.y - ax.y, pos.z - ax.z
    return -x * SCALE, -y * SCALE, z * SCALE

# ── センサーオブジェクト取得 ──────────────────────────────

sonar_obj = bpy.data.objects.get("37316c01.dat")
color_obj = bpy.data.objects.get("37308c01.dat")
sensor_names = {o.name for o in [sonar_obj, color_obj] if o}

# ── radar_dome エクスポート（センサーを除外） ─────────────
# rotor_axis_obj = bevel_gear_36 EMPTY（旧版の32498.datと同じ旋回軸位置）

print("\n" + "=" * 60)
print("radar_dome 処理（センサー除外）")
print("=" * 60)

print(f"  旋回軸オブジェクト(bevel_gear_36 EMPTY): {'OK' if gear36_root else 'NOT FOUND → center にフォールバック'}")

dome_meshes_no_sensor = [m for m in dome_meshes if m.name not in sensor_names]
print(f"  センサー除外: {len(dome_meshes)} → {len(dome_meshes_no_sensor)} MESH")

dome_result = export_stl(
    meshes         = dome_meshes_no_sensor,
    name           = "radar_dome",
    center_mode    = "rotor_axis" if gear36_root else "center",
    rotate_z_deg   = 0.0,
    rotor_axis_obj = gear36_root,
    out_filename   = "radar_dome.stl",
)

# ── starter 構造パーツ STL エクスポート ────────────────────────────────
# press_block と force sensor 部品を除く LEGO 構造パーツのみを出力

print("\n" + "=" * 60)
print("starter_base 処理（LEGO構造パーツ、sensor/press_blockを除く）")
print("=" * 60)

if starter_root:
    # press_block 配下は除外する
    press_block_names = set()
    if press_block_obj:
        press_block_names.add(press_block_obj.name)
        press_block_names.update(c.name for c in press_block_obj.children_recursive)

    starter_structure = []
    for child in starter_root.children_recursive:
        if child.type == 'MESH' and child.name not in press_block_names:
            starter_structure.append(child)

    if starter_structure:
        export_stl(
            meshes       = starter_structure,
            name         = "starter_base",
            center_mode  = "bottom_z",
            out_filename = "starter_base.stl",
        )
    else:
        print("  WARNING: starter 構造パーツが見つかりません")
else:
    print("  WARNING: starter オブジェクトが見つかりません")

# ── センサー: STL出力なし、コンポーネント用マウント pos のみ計算 ──────

print("\n" + "=" * 60)
print("センサーコンポーネント用マウント pos 計算")
print("  (STLは libspikehat_sim/examples/meshes/ のものを使用)")
print("=" * 60)

def compute_component_mount_pos(sensor_meshes, sensor_obj, gear36_root, stl_sensor_local_ldu):
    """
    libspikehat_sim STL の local(0,0,0) が対応する dome-local MuJoCo 座標を計算する。

    stl_sensor_local_ldu: スタンドアロンモデルで求めた STL(0,0,0) のセンサー Blender ローカル座標 (LDU)
      = standalone bbox の (cx, cy, z0) - センサーオブジェクトのスタンドアロン Blender world 位置
    """
    stl_local = mathutils.Vector(stl_sensor_local_ldu)
    stl_world = sensor_obj.matrix_world @ stl_local
    ax = gear36_root.matrix_world.translation
    dx, dy, dz = stl_world.x - ax.x, stl_world.y - ax.y, stl_world.z - ax.z
    return -dx * SCALE, -dy * SCALE, dz * SCALE

if gear36_root:
    if sonar_obj:
        spx, spy, spz = compute_local_body_pos(sonar_obj, gear36_root)
        print(f'  [参考] 距離センサー LDraw原点 dome-local: pos="{spx:.4f} {spy:.4f} {spz:.4f}"')
        # STL(0,0,0) はスタンドアロン Z_min に対応 (sensor local offset: (0,-29.4,-28.9) LDU)
        # スタンドアロン: 37316c01.dat at LDR(-39,-29,28) → Blender(-39,28,29)
        # standalone bbox: cx=-39, cy=-1.4, z0=0.1 → STL_local = (0,-29.4,-28.9)
        smx, smy, smz = compute_component_mount_pos(
            [sonar_obj], sonar_obj, gear36_root, (0, -29.4, -28.9))
        print(f'  距離センサー STL(0,0,0) dome-local mount pos="{smx:.4f} {smy:.4f} {smz:.4f}"')
        print(f'    → <body name="sonar_sensor" pos="{smx:.4f} {smy:.4f} {smz:.4f}" euler="180 0 0">')
    else:
        print("  WARNING: 37316c01.dat (距離センサー) が見つかりません")

    if color_obj:
        cpx, cpy, cpz = compute_local_body_pos(color_obj, gear36_root)
        print(f'  [参考] カラーセンサー LDraw原点 dome-local: pos="{cpx:.4f} {cpy:.4f} {cpz:.4f}"')
        # STL(0,0,0) はスタンドアロン Z_min に対応 (sensor local offset: (0,-19.4,-28.9) LDU)
        # スタンドアロン: 37308.dat at LDR(-25,-29,-32) → Blender(-25,-32,29)
        # standalone bbox: cx=-25, cy=-51.4, z0=0.1 → STL_local = (0,-19.4,-28.9)
        cmx, cmy, cmz = compute_component_mount_pos(
            [color_obj], color_obj, gear36_root, (0, -19.4, -28.9))
        print(f'  カラーセンサー STL(0,0,0) dome-local mount pos="{cmx:.4f} {cmy:.4f} {cmz:.4f}"')
        print(f'    → <body name="color_sensor" pos="{cmx:.4f} {cmy:.4f} {cmz:.4f}" euler="-90 180 0">')
else:
    print("  WARNING: bevel_gear_36 が見つかりません")

# ── obstacle_wall_a エクスポート ──────────────────────────

print("\n" + "=" * 60)
print("obstacle_wall_a 処理")
print("=" * 60)

export_stl(
    meshes       = wall_a_meshes,
    name         = "obstacle_wall_a",
    center_mode  = "bottom_z",
    out_filename = "obstacle_wall_a.stl",
)

# ── obstacle_wall_b エクスポート ──────────────────────────

print("\n" + "=" * 60)
print("obstacle_wall_b 処理")
print("=" * 60)

export_stl(
    meshes       = wall_b_meshes,
    name         = "obstacle_wall_b",
    center_mode  = "bottom_z",
    out_filename = "obstacle_wall_b.stl",
)

# ── MJCF pos 計算 ─────────────────────────────────────────

print("\n" + "=" * 60)
print("MJCF pos 値")
print("=" * 60)

if all_base_meshes and gear36_root:
    (bx0,bx1),(by0,by1),(bz0,bz1) = combined_bbox(all_base_meshes)
    bdx = -(bx0+bx1)/2
    bdy = -(by0+by1)/2
    bdz = -bz0

    ax36 = gear36_root.matrix_world.translation
    g36_mx = -(ax36.x + bdx) * SCALE
    g36_my = -(ax36.y + bdy) * SCALE
    g36_mz =  (ax36.z + bdz) * SCALE

    print(f"  bevel_gear_36 pos: \"{g36_mx:.4f} {g36_my:.4f} {g36_mz:.4f}\"")

    if gear12_root:
        ax12 = gear12_root.matrix_world.translation
        g12_mx = -(ax12.x + bdx) * SCALE
        g12_my = -(ax12.y + bdy) * SCALE
        g12_mz =  (ax12.z + bdz) * SCALE
        print(f"  motor_rotor  pos: \"{g12_mx:.4f} {g12_my:.4f} {g12_mz:.4f}\"")

    print(f"\n--- XML snippet ---")
    print(f'<!-- モーター側 12T ギア -->')
    if gear12_root:
        print(f'<body name="motor_rotor" pos="{g12_mx:.4f} {g12_my:.4f} {g12_mz:.4f}" euler="0 0 0">')
        print(f'  <joint name="motor_joint" type="hinge" axis="0 0 -1" damping="0.85" armature="0.001"/>')
        print(f'  <geom name="bevel_gear_12_geom" type="mesh" mesh="bevel_gear_12_mesh"')
        print(f'        contype="0" conaffinity="0" rgba="0.9 0.7 0.1 1"/>')
        print(f'</body>')
    print(f'<!-- ドーム側 36T ギア（旋回軸）→ radar_dome を子bodyとして保持 -->')
    print(f'<body name="bevel_gear_36" pos="{g36_mx:.4f} {g36_my:.4f} {g36_mz:.4f}" euler="0 0 0">')
    print(f'  <inertial pos="0 0 0" mass="0.05" diaginertia="0.0005 0.0005 0.0005"/>')
    print(f'  <joint name="dome_joint" type="hinge" axis="0 0 -1" damping="0.1" armature="0.0005"/>')
    print(f'  <geom name="bevel_gear_36_geom" type="mesh" mesh="bevel_gear_36_mesh"')
    print(f'        contype="0" conaffinity="0" rgba="0.9 0.7 0.1 1"/>')
    print(f'  <!-- radar_dome は 36T ギア軸に固定された子body -->')
    print(f'  <body name="radar_dome" pos="0 0 0" euler="0 0 0">')
    print(f'    <inertial pos="0 0 0" mass="0.1" diaginertia="0.001 0.001 0.001"/>')
    print(f'    <geom name="dome_geom" type="mesh" mesh="radar_dome_mesh"')
    print(f'          contype="0" conaffinity="0" rgba="0.2 0.5 0.2 1"/>')
    if sonar_obj and gear36_root:
        smx, smy, smz = compute_component_mount_pos(
            [sonar_obj], sonar_obj, gear36_root, (0, -29.4, -28.9))
        print(f'    <!-- 距離センサー: libspikehat_sim distance_sensor_body コンポーネント')
        print(f'         euler="180 0 0": 検出面(+Y)→dome -Y(外向き), 高さ(+Z)→dome -Z -->')
        print(f'    <body name="sonar_sensor" pos="{smx:.4f} {smy:.4f} {smz:.4f}" euler="180 0 0">')
        print(f'      <include file="../sim/libspikehat_sim/examples/components/distance_sensor_body.xml"/>')
        print(f'    </body>')
    if color_obj and gear36_root:
        cmx, cmy, cmz = compute_component_mount_pos(
            [color_obj], color_obj, gear36_root, (0, -19.4, -28.9))
        print(f'    <!-- カラーセンサー: libspikehat_sim color_sensor_body コンポーネント')
        print(f'         euler="-90 180 0": 検出面(+Y)→dome -Z(下向き), 高さ(+Z)→dome -Y -->')
        print(f'    <body name="color_sensor" pos="{cmx:.4f} {cmy:.4f} {cmz:.4f}" euler="-90 180 0">')
        print(f'      <include file="../sim/libspikehat_sim/examples/components/color_sensor_body.xml"/>')
        print(f'    </body>')
    print(f'  </body>')
    print(f'</body>')

    # ── starter (sensor_mount / press_block) 世界座標 ────────────────────
    print(f'\n<!-- starter（フォースセンサーユニット）世界 MuJoCo 座標 -->')
    if starter_root:
        stx = -(starter_root.matrix_world.translation.x + bdx) * SCALE
        sty = -(starter_root.matrix_world.translation.y + bdy) * SCALE
        stz =  (starter_root.matrix_world.translation.z + bdz) * SCALE
        print(f'<!-- sensor_mount  pos="{stx:.4f} {sty:.4f} {stz:.4f}" -->')
        print(f'<body name="sensor_mount" pos="{stx:.4f} {sty:.4f} {stz:.4f}">')
        print(f'  <include file="../sim/libspikehat_sim/examples/components/force_sensor_body.xml"/>')
        print(f'</body>')
        if press_block_obj:
            pbx = -(press_block_obj.matrix_world.translation.x + bdx) * SCALE
            pby = -(press_block_obj.matrix_world.translation.y + bdy) * SCALE
            pbz =  (press_block_obj.matrix_world.translation.z + bdz) * SCALE
            print(f'<!-- press_block  pos="{pbx:.4f} {pby:.4f} {pbz:.4f}" -->')
            print(f'<body name="press_block" pos="{pbx:.4f} {pby:.4f} {pbz:.4f}">')
            print(f'  <!-- press_slide joint をここに記述 -->')
            print(f'</body>')
        else:
            print(f'  <!-- press_block: "press_block copy" オブジェクトが見つかりません -->')
    else:
        print(f'  <!-- starter オブジェクトが見つかりません -->')
else:
    print("  pos計算スキップ（必要なオブジェクトが見つからなかった）")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
