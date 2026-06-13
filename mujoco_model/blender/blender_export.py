"""
Blender用スクリプト: STL形式で直接書き出し（numpy-stl使用）

対応モデル: sonar_radar06.io
  Blenderシーン上のオブジェクト階層:
    radar_base   … 台座グレーパーツ（マーカーを含まない）
    marker_red   … 赤マーカーブロック（39789系）
    marker_blue  … 青マーカーブロック（39789系）
    radar_dome   … ドーム＋モーター一式
    motor_body   … モーターボディ
    motor_rotor  … モーターローター

アプローチ:
  - matrix_world で頂点をワールド座標に変換
  - オフセット（XY中心=0, Z底面=0 or XYZ中心=0）を適用
  - Z軸回転を適用（LDraw空間内）
  - LDraw → MuJoCo 座標変換 + SCALE適用:
      mj_x = -rx * SCALE
      mj_y = -ry * SCALE
      mj_z =  rz * SCALE
  向きの調整は MJCF の euler で行う
  - quad は fan 三角形分割してSTLに書き出す

出力:
  radar_base_gray.stl  … 台座グレーパーツ
  radar_base_red.stl   … 赤マーカーブロック
  radar_base_blue.stl  … 青マーカーブロック
  radar_dome.stl
  motor_body.stl
  motor_rotor.stl

MJCF: scale="1 1 1"

使い方:
  Blenderのスクリプトエディタで開き「スクリプトを実行」
"""

import bpy
import mathutils
import math
import os
import sys
import numpy as np
from stl import mesh as stl_mesh

OUTPUT_DIR = "/Users/kuboaki/Documents/projects/sonar_radar/mujoco_model/meshes"
LOG_PATH   = "/Users/kuboaki/Documents/projects/sonar_radar/mujoco_model/blender/blender_export_log.txt"
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
    """全メッシュのワールド頂点から統合bboxを返す"""
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

    # bbox からオフセット計算
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

    # 全メッシュの頂点・三角面を収集
    all_verts = []   # MuJoCo座標 [mj_x, mj_y, mj_z]
    triangles = []   # [vi0, vi1, vi2] (global index)
    v_offset = 0

    for obj in meshes:
        mat = obj.matrix_world
        wv_list = [mat @ v.co for v in obj.data.vertices]

        for v in wv_list:
            # オフセット適用（LDraw空間）
            x, y, z = v.x + dx, v.y + dy, v.z + dz
            # Z軸回転（LDraw空間）
            rx = cos_a * x - sin_a * y
            ry = sin_a * x + cos_a * y
            rz = z
            # LDraw → MuJoCo 変換 + SCALE
            mj_x = -rx * SCALE
            mj_y = -ry * SCALE
            mj_z =  rz * SCALE
            all_verts.append([mj_x, mj_y, mj_z])

        # quad → fan 三角形分割
        for poly in obj.data.polygons:
            vlist = list(poly.vertices)
            for i in range(1, len(vlist) - 1):
                triangles.append([
                    v_offset + vlist[0],
                    v_offset + vlist[i],
                    v_offset + vlist[i + 1],
                ])
        v_offset += len(wv_list)

    # STL書き出し
    stl_data = stl_mesh.Mesh(np.zeros(len(triangles), dtype=stl_mesh.Mesh.dtype))
    for i, tri in enumerate(triangles):
        for j, vi in enumerate(tri):
            stl_data.vectors[i][j] = all_verts[vi]

    filepath = os.path.join(OUTPUT_DIR, out_filename)
    stl_data.save(filepath)
    print(f"  Saved: {filepath}")
    print(f"  頂点数: {len(all_verts)}  三角面数: {len(triangles)}")

    half_Z = (y1 - y0) / 2 * SCALE
    mj_z_min = -(y1 + dy) * SCALE
    mj_z_max = -(y0 + dy) * SCALE
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

base_root   = bpy.data.objects.get("radar_base")
red_root    = bpy.data.objects.get("marker_red")
blue_root   = bpy.data.objects.get("marker_blue")
dome_root   = bpy.data.objects.get("radar_dome")
gear12_root = bpy.data.objects.get("bevel_gear_12")

base_meshes  = collect_mesh_descendants(base_root,  stop_at_empty=True)  if base_root  else []
red_meshes   = collect_mesh_descendants(red_root)   if red_root   else []
blue_meshes  = collect_mesh_descendants(blue_root)  if blue_root  else []
dome_meshes  = collect_mesh_descendants(dome_root)  if dome_root  else []
gear12_meshes = collect_mesh_descendants(gear12_root) if gear12_root else []

print(f"\n収集結果:")
print(f"  radar_base  = {len(base_meshes)} MESH")
print(f"  marker_red  = {len(red_meshes)} MESH  {'OK' if red_root else 'NOT FOUND'}")
print(f"  marker_blue = {len(blue_meshes)} MESH  {'OK' if blue_root else 'NOT FOUND'}")
print(f"  radar_dome  = {len(dome_meshes)} MESH")
print(f"  bevel_gear_12 = {len(gear12_meshes)} MESH  {'OK' if gear12_root else 'NOT FOUND'}")


# ── radar_base 共有オフセット計算 ─────────────────────────
# gray/red/blue の3つは同じオフセットを使う（位置関係を保つため）
# 全baseパーツ（gray+red+blue）の統合bboxから計算

print("\n" + "=" * 60)
print("radar_base 共有オフセット計算")
print("=" * 60)

all_base_meshes = base_meshes + red_meshes + blue_meshes
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


# ── radar_base_gray エクスポート ──────────────────────────

print("\n" + "=" * 60)
print("radar_base_gray 処理")
print("=" * 60)

base_half_Z = export_stl(
    meshes         = base_meshes,
    name           = "radar_base_gray",
    center_mode    = "shared",
    shared_offset  = base_shared_offset,
    rotate_z_deg   = 0.0,
    out_filename   = "radar_base_gray.stl",
)


# ── marker_red エクスポート ───────────────────────────────

print("\n" + "=" * 60)
print("marker_red 処理")
print("=" * 60)

export_stl(
    meshes         = red_meshes,
    name           = "marker_red",
    center_mode    = "shared",
    shared_offset  = base_shared_offset,
    rotate_z_deg   = 0.0,
    out_filename   = "radar_base_red.stl",
)


# ── marker_blue エクスポート ──────────────────────────────

print("\n" + "=" * 60)
print("marker_blue 処理")
print("=" * 60)

export_stl(
    meshes         = blue_meshes,
    name           = "marker_blue",
    center_mode    = "shared",
    shared_offset  = base_shared_offset,
    rotate_z_deg   = 0.0,
    out_filename   = "radar_base_blue.stl",
)


# ── bevel_gear_12 エクスポート ────────────────────────────

print("\n" + "=" * 60)
print("bevel_gear_12 処理")
print("=" * 60)

export_stl(
    meshes         = gear12_meshes,
    name           = "bevel_gear_12",
    center_mode    = "rotor_axis" if gear12_root else "center",
    rotate_z_deg   = 0.0,
    rotor_axis_obj = gear12_root,
    out_filename   = "bevel_gear_12.stl",
)


# ── radar_dome エクスポート ───────────────────────────────

print("\n" + "=" * 60)
print("radar_dome 処理")
print("=" * 60)

rotor_axis_obj = bpy.data.objects.get("37316c01.dat")
print(f"  旋回軸オブジェクト(37316c01.dat): {'OK' if rotor_axis_obj else 'NOT FOUND → center にフォールバック'}")

dome_result = export_stl(
    meshes         = dome_meshes,
    name           = "radar_dome",
    center_mode    = "rotor_axis" if rotor_axis_obj else "center",
    rotate_z_deg   = 180.0,
    rotor_axis_obj = rotor_axis_obj,
    out_filename   = "radar_dome.stl",
)


# ── MJCF pos 計算 ─────────────────────────────────────────

print("\n" + "=" * 60)
print("MJCF pos 値")
print("=" * 60)

if base_half_Z is not None and dome_result is not None:
    base_pos_z = base_half_Z

    if rotor_axis_obj:
        ax = rotor_axis_obj.matrix_world.translation
        (bx0,bx1),(by0,by1),(bz0,bz1) = combined_bbox(all_base_meshes)
        bdx = -(bx0+bx1)/2
        bdy = -(by0+by1)/2
        bdz = -bz0
        rx = ax.x + bdx
        ry = ax.y + bdy
        rz = ax.z + bdz
        dome_mx = -rx * SCALE
        dome_my = -ry * SCALE
        dome_mz =  rz * SCALE

        from stl import mesh as stl_mesh2
        import os as _os
        _dome_path = _os.path.join(OUTPUT_DIR, "radar_dome.stl")
        try:
            _dm = stl_mesh2.Mesh.from_file(_dome_path)
            _vz = _dm.vectors.reshape(-1,3)[:,2]
            site_z = float(_vz.max())
        except Exception:
            site_z = 0.020

        print(f"  旋回軸モード: モーター位置から dome pos を計算")
        print(f"  radar_base pos : \"0 0 0\"")
        print(f"  radar_dome pos : \"{dome_mx:.4f} {dome_my:.4f} {dome_mz:.4f}\"")
        print(f"  sonar_site pos : \"0 0 {site_z:.4f}\"")
        print(f"\n--- XML snippet ---")
        print(f'<body name="radar_base" pos="0 0 0" euler="0 0 0">')
        print(f'  <inertial pos="0 0 0" mass="1.0" diaginertia="0.01 0.01 0.01"/>')
        print(f'  <geom name="base_gray_geom" type="mesh" mesh="radar_base_gray_mesh"')
        print(f'        contype="0" conaffinity="0" rgba="0.366 0.361 0.371 1"/>')
        print(f'  <geom name="base_red_geom" type="mesh" mesh="radar_base_red_mesh"')
        print(f'        contype="0" conaffinity="0" rgba="0.578 0.010 0.002 1"/>')
        print(f'  <geom name="base_blue_geom" type="mesh" mesh="radar_base_blue_mesh"')
        print(f'        contype="0" conaffinity="0" rgba="0.000 0.089 0.515 1"/>')
        print(f'</body>')
    else:
        print(f"  radar_base pos : \"0 0 0\"")
        print(f"  ※ rotor_axis_obj が見つからなかったため dome pos は手動で調整してください")
else:
    print("  pos計算スキップ（オブジェクトが見つからなかった）")

print(f"\n# ログ出力完了: {LOG_PATH}")
_tee.close()
