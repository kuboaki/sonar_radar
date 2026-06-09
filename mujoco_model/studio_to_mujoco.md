# Bricklink StudioのモデルをMuJoCoで使う

## 概要

Bricklink Studioで作成したLEGOモデルをMuJoCo物理シミュレーターで使うための手順をまとめる。
ここでは、ソナーレーダーモデル（`sonar_radar`）を例に説明する。

```
Studio (.io) → Blender → STLメッシュ → MuJoCo XML → MuJoCoビューア
```

---

## 1. Bricklink Studioでモデルを作成する

### 1.1 サブモデルの設計方針

MuJoCoで扱いやすくするため、Studioのモデル設計時に以下の点を考慮する。

**可動部はサブモデルに分ける**

MuJoCoのジョイント（関節）は body の階層で表現する。Studioでも同様に、動く部分を別サブモデルとして分離しておく。

例：
- `radar_base` … 台座（固定）
- `radar_dome` … ドームとモーター一式（旋回する）

**色を使う部分（マーカーなど）もサブモデルに分ける**

MuJoCoのSTLメッシュは単色しか指定できない。複数色を再現するには、色ごとにSTLを分けてgeomを複数定義する必要がある。そのため、色の異なるパーツは別サブモデルとして分離しておく。

例：
- `marker_red` … 赤マーカーブロック（`39789.dat` + ブッシュ `3713.dat`）
- `marker_blue` … 青マーカーブロック（`39789.dat` + ブッシュ `3713.dat`）

> **注意：** Studioでは単一ブロックのサブモデルは作成できない。ブッシュなど固定パーツと組み合わせてサブモデルにする。

**モーターのローターはカスタムパーツ化する**

標準のモーターパーツはボディとローターが一体のため、MuJoCoで回転させられない。以下のように分離したカスタムパーツを作成する。

- `motor_body`（`m71d95b20_202662_110634.dat`）… モーター本体（固定）
- `motor_rotor`（`m71d95b20_202662_032825.dat`）… ターンテーブル（旋回軸）

ローターのdat原点が旋回軸の位置になるようにカスタムパーツを設計することが重要。

### 1.2 サブモデルの構造例

```
sonar_radar06.io
├── radar_base         … 台座グレーパーツ
│   ├── marker_blue    … 青マーカー（サブモデル）
│   │   ├── 3713.dat   … ブッシュ（グレー）
│   │   └── 39789.dat  … マーカーブロック（青）
│   └── marker_red     … 赤マーカー（サブモデル）
│       ├── 3713.dat   … ブッシュ（グレー）
│       └── 39789.dat  … マーカーブロック（赤）
└── radar_dome         … ドーム＋モーター一式
    ├── motor_body     … モーター本体
    ├── motor_rotor    … ターンテーブル（旋回軸）
    └── （その他パーツ）
```

---

## 2. Blenderの準備

### 2.1 使用するアドオン

**ldr_tools_blender**（by ScanMountGoat）

- リポジトリ：https://github.com/ScanMountGoat/ldr_tools_blender
- LDraw形式（`.ldr`、`.mpd`）およびBricklink Studioの `.io` ファイルをBlenderにインポートするアドオン
- Blender 4.1以降に対応（リリースはBlenderのPythonバージョンに紐付き）
- Rustベースの高速実装

**インストール手順：**

1. GitHubの [Releases](https://github.com/ScanMountGoat/ldr_tools_blender/releases) ページから使用しているBlenderバージョンに対応したzipをダウンロード（zipは展開しない）
2. Blenderを起動し、`Edit → Preferences → Add-ons → Install` を開く
3. ダウンロードしたzipファイルを選択してインストール
4. アドオン一覧で `ldr_tools_blender` を検索してチェックを入れて有効化

> **アップグレード時の注意：** 旧バージョンをアンインストールしてから新バージョンをインストールする必要がある。

### 2.2 デフォルトオブジェクトの削除

Blenderを新規起動すると、デフォルトでカメラ・ライト・キューブが挿入されている。ioファイルをインポートする前に、これらをすべて削除しておく。

```
Edit → Delete All（またはシーン内のオブジェクトをすべて選択してDelete）
```

### 2.3 アドオンの設定（ldr_tools_blender）

`File → Import → LDraw` を選択すると、ファイルダイアログの左側にインポートオプションパネルが表示される。以下の項目を設定する。

#### LDraw Library（LDrawライブラリのパス）

LDrawパーツライブラリの場所を指定する。Studioのアプリケーションフォルダ内のものをそのまま使用できるため、別途ダウンロードは不要。

- Mac: `/Applications/Studio 2.0/ldraw`

#### Additional Library Path（追加ライブラリのパス）

Studioで作成したカスタムパーツ（モーターローターなど）の格納場所を指定する。

- Mac: `/Users/<ユーザー名>/.local/share/Studio.io/CustomParts/`

> Macでは `.local` は隠しディレクトリのためFinderでは非表示。パス入力欄に直接入力する。

#### その他のパラメータ

| パラメータ | 設定値 | 備考 |
|-----------|--------|------|
| Instance Type | **Linked Duplicates** | 同種パーツがメッシュを共有し軽量化される |
| Stud Type | **Normal** | 標準のスタッド形状 |
| Resolution | **Low** | ポリゴン数を抑える（MuJoCo用途では十分） |
| Gap Between Parts | **チェックあり** | パーツ間に微小な隙間を入れる（実物に近い見た目） |
| Scale | **1.0** | スケール変換はエクスポートスクリプト側で行うため、Blender上は原寸のまま |

> **Scaleを1.0にする理由：** Blenderインポート時にスケールを変えると `matrix_world` による頂点座標変換が複雑になる。エクスポートスクリプト内で `SCALE = 0.0004`（LDU→メートル）を適用するため、Blender上は原寸（LDU単位）のまま扱う。

---

## 3. BlenderへのioファイルのインポートとSTL生成

### 3.1 インポート手順

1. Blenderを起動し、デフォルトオブジェクトを削除
2. `File → Import → LDraw` で `.io` ファイルを選択
3. Scale=1 でインポート
4. アウトライナーでサブモデルの階層構造を確認

### 3.2 インポート後のオブジェクト階層

インポートすると、Studioのサブモデル構造がBlenderのオブジェクト階層として反映される。

```
sonar_radar06.io  [EMPTY]
├── radar_base    [EMPTY]
│   ├── 32054.dat [MESH]
│   ├── ...
│   ├── marker_blue [EMPTY]
│   │   ├── 3713.dat  [MESH]
│   │   └── 39789.dat [MESH]
│   └── marker_red  [EMPTY]
│       ├── 3713.dat.001  [MESH]
│       └── 39789.dat.001 [MESH]
└── radar_dome    [EMPTY]
    ├── 37316c01.dat [MESH]  ← 旋回軸
    └── ...
```

---

## 4. Blenderスクリプトによる変換（blender_export.py）

### 4.1 スクリプトの概要

`blender_export.py` をBlenderのスクリプトエディタで実行することで、MuJoCo用のSTLメッシュファイルを生成する。

### 4.2 座標変換の処理

LDraw座標系からMuJoCo座標系への変換を以下の手順で行う。

1. **ワールド座標に変換**  
   `matrix_world` を使って各頂点をBlenderのワールド座標に変換する

2. **オフセット適用（XY中心化・Z底面=0）**  
   全パーツの統合バウンディングボックスを計算し、XY方向は中心、Z方向は底面が0になるようにオフセットを加算する
   ```python
   dx, dy, dz = -cx, -cy, -z0   # bottom_z モード
   ```

3. **LDraw → MuJoCo座標変換 + スケール適用**  
   ```python
   mj_x = -rx * SCALE   # SCALE = 0.0004 (LDU → m)
   mj_y = -ry * SCALE
   mj_z =  rz * SCALE
   ```

4. **quad → 三角形分割**  
   MuJoCo（STL）は三角形ポリゴンのみ対応のため、四角形ポリゴンをfan分割する

### 4.3 サブモデル境界の扱い（stop_at_empty）

`radar_base` のSTL生成時に `marker_red`/`marker_blue` のメッシュを誤って含めないよう、`stop_at_empty=True` を指定する。これにより、EMPTY（サブモデル境界）に到達したところで再帰収集を停止する。

```python
def collect_mesh_descendants(root, stop_at_empty=False):
    for child in root.children:
        if child.type == 'MESH':
            result.append(child)
        elif child.type == 'EMPTY':
            if stop_at_empty:
                pass  # サブモデル境界でストップ
            else:
                result.extend(collect_mesh_descendants(child, stop_at_empty))
```

### 4.4 色別STL分割

マーカーブロックの色を再現するため、`radar_base` を3つのSTLに分割する。

**共有オフセットを使う理由：**
gray/red/blueの3つのSTLは、全パーツの統合bboxから計算した共通オフセットを使う。これにより3つのSTLの位置関係が正しく保たれ、MuJoCo上で重ね合わせたときに一致する。

### 4.5 旋回軸の扱い

`radar_dome` のSTL生成時は、旋回軸オブジェクト（`37316c01.dat`）のワールド座標を原点として使う（`center_mode="rotor_axis"`）。これにより、STLの原点がMuJoCoのジョイント位置と一致する。

### 4.6 スクリプトの実行方法

1. Blenderのスクリプトエディタで `blender_export.py` を開く
2. 「スクリプトを実行」ボタンをクリック
3. `blender_export_log.txt` で結果を確認

---

## 5. 生成されるファイル

### 5.1 STLメッシュファイル（meshes/フォルダ）

| ファイル | 内容 | rgba |
|----------|------|------|
| `radar_base_gray.stl` | 台座グレーパーツ（11パーツ） | `0.366 0.361 0.371 1` |
| `radar_base_red.stl` | 赤マーカーブロック（2パーツ） | `0.578 0.010 0.002 1` |
| `radar_base_blue.stl` | 青マーカーブロック（2パーツ） | `0.000 0.089 0.515 1` |
| `radar_dome.stl` | ドーム+モーター一式 | `0.2 0.5 0.2 1` |
| `motor_body.stl` | モーター本体 | `0.5 0.5 0.5 1` |
| `motor_rotor.stl` | ターンテーブル（旋回軸） | `0.8 0.4 0.1 1` |

### 5.2 ログファイル

- `blender_export_log.txt` … 座標変換のオフセット値、MJCF posの計算結果、XMLスニペットを出力

---

## 6. MuJoCo XMLの構造

### 6.1 ioファイルのサブモデルとXMLの対応

Studioのサブモデル構造がMuJoCoのbody階層に対応する。

```
Studio サブモデル         MuJoCo XML
─────────────────────    ──────────────────────────
radar_base            →  <body name="radar_base">
  marker_red          →    <geom name="base_red_geom" .../>  ※同じbody内に複数geom
  marker_blue         →    <geom name="base_blue_geom" .../>
radar_dome            →  <body name="radar_dome">
  motor_rotor         →    <body name="motor_rotor">
                               <joint name="turret_joint" .../>  ← 旋回軸
```

### 6.2 XMLの構造

```xml
<mujoco model="sonar_radar">
  <compiler angle="degree" meshdir="meshes/"/>

  <asset>
    <!-- 色別に分割したSTL -->
    <mesh name="radar_base_gray_mesh" file="radar_base_gray.stl" scale="1 1 1"/>
    <mesh name="radar_base_red_mesh"  file="radar_base_red.stl"  scale="1 1 1"/>
    <mesh name="radar_base_blue_mesh" file="radar_base_blue.stl" scale="1 1 1"/>
    <mesh name="radar_dome_mesh"      file="radar_dome.stl"      scale="1 1 1"/>
    <mesh name="motor_body_mesh"      file="motor_body.stl"      scale="1 1 1"/>
    <mesh name="motor_rotor_mesh"     file="motor_rotor.stl"     scale="1 1 1"/>
  </asset>

  <worldbody>
    <!-- 台座：色別に複数geomで定義 -->
    <body name="radar_base" pos="0 0 0" euler="0 0 0">
      <geom name="base_gray_geom" type="mesh" mesh="radar_base_gray_mesh"
            contype="0" conaffinity="0" rgba="0.366 0.361 0.371 1"/>
      <geom name="base_red_geom"  type="mesh" mesh="radar_base_red_mesh"
            contype="0" conaffinity="0" rgba="0.578 0.010 0.002 1"/>
      <geom name="base_blue_geom" type="mesh" mesh="radar_base_blue_mesh"
            contype="0" conaffinity="0" rgba="0.000 0.089 0.515 1"/>

      <!-- モーターボディ（固定） -->
      <body name="motor_body" pos="0.0076 0.0004 0.0">
        <geom name="motor_body_geom" type="mesh" mesh="motor_body_mesh"
              contype="0" conaffinity="0" rgba="0.5 0.5 0.5 1"/>

        <!-- モーターローター（旋回） -->
        <body name="motor_rotor" pos="0.0001 0.0239 0.0160">
          <joint name="turret_joint" type="hinge" axis="0 0 1"
                 damping="0.05" armature="0.001"/>
          <geom name="motor_rotor_geom" type="mesh" mesh="motor_rotor_mesh"
                contype="0" conaffinity="0" rgba="0.8 0.4 0.1 1"/>

          <!-- ドーム（rotorに従属して旋回） -->
          <body name="radar_dome" pos="0 0.008 0.028" euler="0 0 180">
            <geom name="dome_geom" type="mesh" mesh="radar_dome_mesh"
                  contype="0" conaffinity="0" rgba="0.2 0.5 0.2 1"/>
            <!-- ソナーセンサー（前方） -->
            <site name="sonar_site"  pos="0 -0.016 0.000" size="0.01" rgba="1 0 0 1"/>
            <!-- カラーセンサー（後方・下向き） -->
            <site name="color_site"  pos="0  0.048 0.008" size="0.01" rgba="1 1 0 1"/>
          </body>
        </body>
      </body>
    </body>
  </worldbody>

  <actuator>
    <motor name="turret_motor" joint="turret_joint"
           gear="10" ctrllimited="true" ctrlrange="-1 1"/>
  </actuator>

  <sensor>
    <jointpos  name="turret_angle" joint="turret_joint"/>
    <jointvel  name="turret_vel"   joint="turret_joint"/>
    <framepos  name="sonar_pos"    objtype="site" objname="sonar_site"/>
    <framequat name="sonar_quat"   objtype="site" objname="sonar_site"/>
    <framepos  name="color_pos"    objtype="site" objname="color_site"/>
    <framequat name="color_quat"   objtype="site" objname="color_site"/>
  </sensor>

</mujoco>
```

### 6.3 確定した位置パラメータ

| body | pos | euler |
|------|-----|-------|
| `radar_base` | `0 0 0` | `0 0 0` |
| `motor_body` | `0.0076 0.0004 0.0` | `0 0 0` |
| `motor_rotor` | `0.0001 0.0239 0.0160` | `0 0 0` |
| `radar_dome` | `0 0.008 0.028` | `0 0 180` |

**センサーsite：**

| site | pos | 用途 |
|------|-----|------|
| `sonar_site` | `0 -0.016 0.000` | ソナーセンサー（前方） |
| `color_site` | `0 0.048 0.008` | カラーセンサー（後方・下向き） |

---

## 7. MuJoCoビューアで確認する

### 7.1 起動方法

```bash
cd /Users/kuboaki/Documents/LEGO_Studio_models/sonar_radar_model/
python -m mujoco.viewer --mjcf=sonar_radar.xml
```

### 7.2 確認ポイント

- **静止状態** … 各パーツが正しい位置に配置されているか
- **地面への接地** … `radar_base` の底面が床（Z=0）に接しているか
- **旋回動作** … Controlスライダーで `turret_motor` を操作し、ドームがrotorと一緒に旋回するか
- **マーカーの色** … 赤・青のマーカーブロックが正しい色で表示されているか

### 7.3 ビューア操作

| 操作 | 内容 |
|------|------|
| マウス右ドラッグ | 視点回転 |
| マウス中ドラッグ | 平行移動 |
| スクロール | ズーム |
| Simulation → Run | シミュレーション開始 |
| Control パネル | actuatorのスライダー操作 |

---

## 8. ファイル一覧

```
sonar_radar_model/
├── sonar_radar.xml              … MuJoCo本番XMLファイル
├── sonar_radar_color_test.xml   … 色確認用テストXML
├── blender_export.py            … Blenderエクスポートスクリプト（本番）
├── blender_export_log.txt       … エクスポートログ
└── meshes/
    ├── radar_base_gray.stl      … 台座グレーパーツ
    ├── radar_base_red.stl       … 赤マーカー
    ├── radar_base_blue.stl      … 青マーカー
    ├── radar_dome.stl           … ドーム
    ├── motor_body.stl           … モーター本体
    └── motor_rotor.stl          … モーターローター
```

---

## 9. libspikehat_sim との連携

### 9.1 概要

`libspikehat_sim` は `libspikehat` と同じAPIを持つMuJoCo版シミュレーターライブラリ。
`sonar_radar.xml` を使うことで、`sonar_radar.py` をそのままシムで動かせる。

```
実機（Raspberry Pi）:  sonar_radar.py → libspikehat    → Build HAT → SPIKE Prime
シム（Mac/Linux）:     sonar_radar.py → libspikehat_sim → MuJoCo   → sonar_radar.xml
```

### 9.2 sonar_radar_sim.py の仕組み

`sonar_radar_sim.py` が `libspikehat_sim` の `SimSpikeHat` を `spikehat.SpikeHat` として差し込む：

```python
# sonar_radar_sim.py
from sim_spikehat import SimSpikeHat
# SimSpikeHat を spikehat.SpikeHat として登録
sys.modules["spikehat"] = ...
# sonar_radar.py をそのまま実行
exec(open("sonar_radar.py").read())
```

### 9.3 siteの役割

MuJoCo XMLに定義したsiteがセンサーの検出点として機能する：

| site | 用途 | libspikehat_sim での使われ方 |
|------|------|---------------------------|
| `sonar_site` | ソナーセンサー位置 | `distance_read`: 前方レイキャスト |
| `color_site` | カラーセンサー位置 | `color_read_hsv/rgb`: 真下レイキャスト |

### 9.4 実行方法

```bash
cd /path/to/sonar_radar/
SPIKEHAT_SIM_XML=sim/sonar_radar.xml python3 sonar_radar_sim.py
```
