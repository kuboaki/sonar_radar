# Bricklink StudioのモデルをMuJoCoで使う

## 概要

Bricklink Studioで作成したLEGOモデルをMuJoCo物理シミュレーターで使うための手順をまとめる。
ここでは、ソナーレーダーモデル（`sonar_radar`）を例に説明する。

```
Studio (.io) → Blender → STLメッシュ → MuJoCo XML（配置調整）→ Control割り当て → ビューア設定
```

最終的な目的は「実機の開発をシミュレータ上のコード(`raspi/sonar_radar.py`)で行い、
そのコードをそのまま実機(`libspikehat`)に持っていって動かす」こと。
そのためMuJoCoモデル側は、`libspikehat_sim` が要求するjoint/site/sensor構成
（[9. libspikehat_simとの連携](#9-libspikehat_simとの連携)参照）に適合させる必要がある。

---

## 1. Bricklink Studioでモデルを作成する

### 1.1 サブモデルの設計方針

MuJoCoで扱いやすくするため、Studioのモデル設計時に以下の点を考慮する。

**可動部はサブモデルに分ける**

MuJoCoのジョイント（関節）は body の階層で表現する。Studioでも同様に、動く部分を別サブモデルとして分離しておく。

例：
- `radar_base` … 台座（固定）
- `radar_dome` … ドーム一式（旋回する）
- `bevel_gear_12` … モーター側の12Tベベルギア（旋回する）

**色を使う部分（マーカーなど）もサブモデルに分ける**

MuJoCoのSTLメッシュは単色しか指定できない。複数色を再現するには、色ごとにSTLを分けてgeomを複数定義する必要がある。そのため、色の異なるパーツは別サブモデルとして分離しておく。

例：
- `marker_red` … 赤マーカーブロック（`39789.dat` + ブッシュ `3713.dat`）
- `marker_blue` … 青マーカーブロック（`39789.dat` + ブッシュ `3713.dat`）

> **注意：** Studioでは単一ブロックのサブモデルは作成できない。ブッシュなど固定パーツと組み合わせてサブモデルにする。

**ギアでつながる回転部品は、それぞれ独立したサブモデルにする**

`sonar_radar07` では、モーター側の12Tベベルギア（`bevel_gear_12`）とドーム側の36Tベベルギア
（ドーム一式に内包）が別々に回転する。MuJoCo側ではこれを2つの独立したhinge jointとし、
`equality`制約でギア比・回転方向を結びつける（[6.3](#63-equality制約によるギア連動)参照）。
そのため、Studio側でもこの2つを別サブモデルとして分離しておく。

> 旧版（`sonar_radar06`）ではモーター本体とローターを一体のカスタムパーツ
> （`motor_body`/`motor_rotor`）として作成していたが、現行版ではモーター本体側の
> メッシュは `libspikehat_sim/examples/test_motor.xml` 側に切り出され、
> `sonar_radar.xml` には含まれない。

### 1.2 サブモデルの構造例（sonar_radar08）

```
sonar_radar08.io
├── radar_base          … 台座グレーパーツ
│   ├── marker_blue     … 青マーカー（サブモデル）
│   │   ├── 3713.dat    … ブッシュ（グレー）
│   │   └── 39789.dat   … マーカーブロック（青）
│   └── marker_red      … 赤マーカー（サブモデル）
│       ├── 3713.dat    … ブッシュ（グレー）
│       └── 39789.dat   … マーカーブロック（赤）
├── radar_dome          … ドーム一式（36Tベベルギア32498.dat、距離/カラーセンサー含む）
├── bevel_gear_12       … モーター側12Tベベルギア（旋回軸=このサブモデル原点）
├── obstacle_wall_a     … 障害物壁A（黄色、独立サブモデル）
└── obstacle_wall_b     … 障害物壁B（黒、独立サブモデル）
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
sonar_radar07.io  [EMPTY]
├── radar_base       [EMPTY]
│   ├── 32054.dat    [MESH]
│   ├── ...
│   ├── marker_blue  [EMPTY]
│   │   ├── 3713.dat  [MESH]
│   │   └── 39789.dat [MESH]
│   └── marker_red   [EMPTY]
│       ├── 3713.dat.001  [MESH]
│       └── 39789.dat.001 [MESH]
├── radar_dome       [EMPTY]
│   ├── 32498.dat    [MESH]  ← 36Tベベルギア（旋回軸原点）
│   ├── 37316c01.dat [MESH]  ← 距離センサー
│   ├── 37308c01.dat [MESH]  ← カラーセンサー
│   └── ...
└── bevel_gear_12    [EMPTY]
    └── ...（12Tベベルギア本体、旋回軸=このEMPTYの原点）
```

---

## 4. Blenderスクリプトによる変換（blender_export.py）

### 4.1 スクリプトの概要

`blender_export.py` をBlenderのスクリプトエディタで実行することで、MuJoCo用のSTLメッシュファイルと、
MJCFに貼り付けるためのpos値・site値をログに出力する。

### 4.2 座標変換の処理

LDraw座標系からMuJoCo座標系への変換を以下の手順で行う。

1. **ワールド座標に変換**
   `matrix_world` を使って各頂点をBlenderのワールド座標に変換する

2. **オフセット適用**
   `center_mode` に応じてオフセットを計算する（詳細は4.3）

3. **LDraw → MuJoCo座標変換 + スケール適用**
   ```python
   mj_x = -rx * SCALE   # SCALE = 0.0004 (LDU → m)
   mj_y = -ry * SCALE
   mj_z =  rz * SCALE
   ```

4. **quad → 三角形分割**
   MuJoCo（STL）は三角形ポリゴンのみ対応のため、四角形ポリゴンをfan分割する

### 4.3 オフセットの種類（center_mode）

| center_mode | 内容 | 用途 |
|---|---|---|
| `bottom_z` | XY中心化＋Z底面=0 | 単独の固定パーツ |
| `center` | XYZ中心化 | フォールバック |
| `rotor_axis` | 指定オブジェクトの原点を基準 | 旋回するパーツ（`radar_dome`, `bevel_gear_12`） |
| `shared` | 呼び出し側で計算した共通オフセットを使用 | `radar_base`（gray/red/blue で位置関係を保つ） |

`radar_dome` は `32498.dat`（36Tベベルギア）、`bevel_gear_12` はサブモデル自身の原点を
`rotor_axis_obj` として渡す。これにより、STLの原点がそのままMuJoCoの `joint` 位置（hingeの回転軸）
と一致するため、MJCF側で `body pos` を「joint位置」として扱える。

### 4.4 サブモデル境界の扱い（stop_at_empty）

`radar_base` のSTL生成時は、`marker_red`/`marker_blue` のメッシュを誤って含めないよう
`stop_at_empty=True` を指定する。これにより、EMPTY（サブモデル境界）に到達したところで
再帰収集を停止する。

### 4.5 色別STL分割（radar_base）

マーカーブロックの色を再現するため、`radar_base` を3つのSTL（gray/red/blue）に分割する。
gray/red/blueの3つは、全パーツの統合bboxから計算した共通オフセット（`shared_offset`）を使う。
これにより3つのSTLの位置関係が正しく保たれ、MuJoCo上で重ね合わせたときに一致する。

### 4.6 センサーsite位置の自動計算

`radar_dome` エクスポート時、`32498.dat`（旋回軸）を基準に、距離センサー（`37316c01.dat`）と
カラーセンサー（`37308c01.dat`）のローカル座標を計算し、`<site>` タグの形でログに出力する
（`compute_local_site_pos`）。

```
<site name="sonar_site" pos="..." size="0.01" rgba="1 0 0 1"/>
<site name="color_site" pos="..." size="0.01" rgba="1 1 0 1"/>
```

これは**初期値**であり、実際にはビューアで実機の検出結果と見比べながら微調整する
（[7. センサーsiteの配置調整](#7-センサーsiteの配置調整)参照）。

### 4.7 スクリプトの実行方法

1. Blenderのスクリプトエディタで `blender_export.py` を開く
2. 「スクリプトを実行」ボタンをクリック
3. `blender_export_log.txt` で結果（オフセット値、MJCF pos、site pos、XMLスニペット）を確認

---

## 5. 生成されるファイル

### 5.1 STLメッシュファイル（meshes/フォルダ）

| ファイル | 内容 | rgba |
|----------|------|------|
| `radar_base_gray.stl` | 台座グレーパーツ | `0.366 0.361 0.371 1` |
| `radar_base_red.stl` | マーカーブロック(旧・赤。実機実測(2026-08-11)でチャタリング・誤検出が判明し緑に変更。STL/メッシュ名・geom名`base_red_geom`は据え置き、色のみ`sonar_radar.xml`側で`0.020 0.250 0.200 1`(HSV変換でhue≈167、実機実測のhue158〜170に整合)に上書き) | `0.578 0.010 0.002 1`(旧) |
| `radar_base_blue.stl` | 青マーカーブロック | `0.000 0.089 0.515 1` |
| `radar_dome.stl` | ドーム一式（36Tベベルギア含む） | `0.2 0.5 0.2 1` |
| `bevel_gear_12.stl` | モーター側12Tベベルギア | `0.9 0.7 0.1 1` |

### 5.2 ログファイル

- `blender_export_log.txt` … 座標変換のオフセット値、MJCF posの計算結果、site pos、XMLスニペットを出力

---

## 6. MuJoCo XMLへの組み込みと配置調整

### 6.1 body階層とSTLの対応

```
Studio サブモデル        MuJoCo XML
────────────────────    ──────────────────────────────────────────
radar_base           →  <body name="radar_base">
  marker_red         →    <geom name="base_red_geom" .../>  ※同じbody内に複数geom
  marker_blue        →    <geom name="base_blue_geom" .../>

bevel_gear_12        →  <body name="motor_rotor">           ← motor_joint(hinge)
                            <geom name="bevel_gear_12_geom" .../>

radar_dome           →  <body name="radar_dome">            ← dome_joint(hinge)
                            <geom name="dome_geom" .../>
  37316c01.dat       →    <body name="sonar_sensor" pos="..." euler="...">
                              <include file=".../distance_sensor_body.xml"/>
                            </body>
  37308c01.dat       →    <body name="color_sensor" pos="...">
                              <geom name="color_geom" .../>
                              <site name="color_site" .../>
                            </body>
```

`motor_rotor`（12Tギア）と `radar_dome`（36Tギア）は、いずれも `radar_base` の直下に
**独立した body** として配置し、それぞれに `hinge` jointを持たせる。2つのjointは
equality制約で連動させる（[6.3](#63-equality制約によるギア連動)）。

距離センサー（`37316c01.dat`）は `libspikehat_sim` のコンポーネント
（`distance_sensor_body.xml`）を `<include>` で組み込む。
カラーセンサー（`37308c01.dat`）は現状インラインSTLのまま（今後コンポーネント化予定）。

### 6.2 配置（pos/euler）はBlender出力どおりにならない

`blender_export_log.txt` に出力されるpos値（`rotor_axis`基準で計算した値）は**初期値**であり、
そのままではビューア上でパーツ同士の噛み合いや向きがズレることが多い。
実際には以下のような**手動調整が必須**になる。

- **回転方向（axis符号）の調整**
  Studio/Blender側のZ軸回転と、MuJoCo `euler`・`joint axis` の符号の関係は自明ではない。
  「正方向に回したらどちらに動くべきか」を実機と見比べ、`joint axis="0 0 1"` /
  `axis="0 0 -1"` のどちらにするかを決める
  （[[feedback_mujoco_hinge_axis_vs_equality_sign]] 参照：符号調整は
  equality制約側ではなく hinge axis 側で行うこと）。

- **ギアの噛み合い位置の微調整**
  `motor_rotor`（12T）と `radar_dome`（36T）はそれぞれ別サブモデルとしてエクスポートされた
  原点を基準に配置するが、Studio上の組み立て位置によっては数度ズレた状態でしか噛み合わない
  （sonar_radar07では実測5度のズレがあった）。この機構的なズレは
  `raspi/sonar_radar.py` 側の `SENSOR_HOME_OFFSET` で補正し、**MuJoCoモデル自体は
  組み立て可能な位置であればよい**、という方針にしている。

- **dome本体の位置(pos)**
  `radar_dome` の `pos` は、`bevel_gear_12`（motor_rotor）の旋回軸との相対位置になるよう
  ビューアで動かしながら微調整する。現在値は `pos="-0.0083 0.0007 0.0358"`。

### 6.3 equality制約によるギア連動

12T-36Tベベルギアの噛み合い（ギア比1:3、回転方向反転）は、2つのhinge jointを
`equality`制約の `polycoef` で結びつけて表現する。

```xml
<equality>
  <!-- ベベルギア(12T-36T)噛み合い: dome_joint = +motor_joint / 3
       motor_joint axis="0 0 -1"(正=CW), dome_joint axis="0 0 1"(正=CCW)
       CWモーターに対してCCWドーム → polycoef正 -->
  <joint joint1="dome_joint" joint2="motor_joint" polycoef="0 0.33333333 0 0 0"/>
</equality>
```

**motor/dome の axis 設定（sonar_radar08 実機確認済み）:**

| joint | axis | 正方向の意味 |
|---|---|---|
| `motor_joint`（12T, motor_rotor） | `0 0 -1` | 正 = CW（時計回り）= 実機の正エンコーダ方向 |
| `dome_joint`（36T, bevel_gear_36） | `0 0 1` | 正 = CCW（反時計回り）= 実機の正PWM時のドーム旋回方向 |

実機確認：正PWM → エンコーダ正 → モーターCW → ドームCCW（ベベルギアで逆転）。

> **回転方向の調整は `axis` で行う。** `polycoef` の符号を変えて回転方向を反転させると、
> 一見動くが速度比などで不整合が生じることがある。回転方向の反転は
> `<joint ... axis="0 0 -1" .../>` のように joint の `axis` 側で行い、
> `polycoef` はギア比（絶対値）のみを表す、という分担にする。

### 6.4 衝突ガードジオム

壁がモーター柱やドーム旋回範囲に侵入しないよう、`radar_base` body に薄いスラブ型
ガードジオムを追加する。

```xml
<!-- 高さ4mm（Z=0〜4mm）の薄スラブにして、センサーのレイキャスト高さ（30mm以上）に干渉しない -->
<geom name="radar_column_guard" type="box"
      pos="0.008 0.060 0.002" size="0.030 0.030 0.002"
      contype="1" conaffinity="1" rgba="0.9 0.5 0.1 0.3"/>
<geom name="dome_sweep_guard" type="cylinder"
      pos="0.0076 0.0728 0.002" size="0.040 0.002"
      contype="1" conaffinity="1" rgba="0.5 0.8 0.9 0.3"/>
```

> ガードを全高のboxやcylinderにすると `mj_ray`（NULL geomgroup）がセンサーレイキャストの
> 対象に含まれてしまい、カラーセンサーがマーカーより先にガードに当たる問題が発生する。
> `libspikehat_sim` 側にモデル固有の geomgroup フィルタを追加してはならない
> （汎用ライブラリとしての設計原則）ため、ガード側を薄くして高さで回避する。

---

## 7. センサーsiteの配置調整

`sonar_site`（距離センサー）・`color_site`（カラーセンサー）は、4.6で自動計算した値を初期値として、
以下の手順で実機に合わせて微調整する。

1. **初期配置**: `blender_export_log.txt` 出力値をXMLに貼り付ける
2. **静止状態での距離値の確認**: dome角度0°（正面）で `sonar_site` から壁までの距離を
   simで読み取り、実機の同条件での測定値（例: 実機118mm vs sim116mm）と比較してZ/Y位置を調整
3. **マーカー検出角度の確認**: `color_site` の位置・向きにより、赤/青マーカーを検出する
   dome角度がsim/実機で一致するかを確認し、位置を微調整する
4. **ビューアでの目視確認**: site（球で表示される）が実機の対応する部品（センサーブロック）の
   位置に重なっているかを目視で確認する

site位置の調整は、6.2の本体位置(pos)調整と相互に影響するため、
「本体位置を仮決め → site位置を実測値に合わせる → 旋回させて再確認」を繰り返す。

---

## 8. Controlの割り当て

ビューアの **Control** タブから操作できるアクチュエータと、`libspikehat_sim` が参照するsensorを
以下のように対応づける。

### 8.1 actuator一覧

| actuator | joint | 種別 | 用途 |
|---|---|---|---|
| `turret_motor` | `motor_joint` | motor（トルク, gear=10） | `motor_pwm`/`motor_run_to_position` で旋回 |
| `press_ctrl` | `press_slide` | position | 終了スイッチ(`press_body`)を押す（ビューア操作用） |
| `wall_a_x_ctrl` | `wall_a_x` | position | 壁A（黄色）のX位置（ビューア操作用） |
| `wall_a_y_ctrl` | `wall_a_y` | position | 壁A（黄色）のY位置（ビューア操作用） |
| `wall_b_x_ctrl` | `wall_b_x` | position | 壁B（黒）のX位置（ビューア操作用） |
| `wall_b_y_ctrl` | `wall_b_y` | position | 壁B（黒）のY位置（ビューア操作用） |

`turret_motor` のみがアプリケーションコード（`sonar_radar.py`）から操作される。
他は**ビューアのControlタブから人間が操作する**ためのもので、実機には存在しない
sim専用のオブジェクト（壁・終了スイッチ）を動かすために用意している。

**壁のキーボード操作（sonar_radar_sim.py）:**

| キー | 動作 |
|---|---|
| `1` | 壁A（黄色）を選択 |
| `2` | 壁B（黒）を選択 |
| `←` / `→` | 選択中の壁をX方向に1スタッド（8mm）移動 |
| `↑` / `↓` | 選択中の壁をY方向に1スタッド（8mm）移動 |

MuJoCo ビューアの組み込みショートカット（A/D/W/S/F 等）と競合するため、
WASDキーは使用しない。矢印キーは `key_callback` で `_KEY_RIGHT/LEFT/UP/DOWN`
（GLFW keycode 262〜265）として取得する。

### 8.2 sensor一覧

| sensor | 種別 | 対応するlibspikehat_sim API |
|---|---|---|
| `turret_angle`/`turret_vel` | jointpos/jointvel (`motor_joint`) | `motor_get_position` |
| `dome_angle`/`dome_vel` | jointpos/jointvel (`dome_joint`) | （デバッグ用、API未使用） |
| `distance_pos`/`distance_quat` | framepos/framequat (`distance_site`) | `distance_read`（前方レイキャスト） |
| `color_pos`/`color_quat` | framepos/framequat (`color_site`) | `color_read_hsv`/`color_read_rgb`（下方レイキャスト） |
| `force_touch` | touch (`force_site`) | `force_is_pressed` |

新しい実機を作る場合も、**アプリコードが使うAPIに対応するsite/sensorは必ず用意する**
（`motor_joint` + `sonar_site` + `color_site` + `force_site` 相当）。
壁や終了スイッチのような「sim専用の操作対象」は、実機に存在しなくても
ビューア確認用に追加してよい。

---

## 9. ビューアの設定（sonar_radar_sim.py）

`sim/sonar_radar_sim.py` がMuJoCoモデルとビューアを起動するランチャー。
モデル固有の設定は主に以下の3か所に集中している。

### 9.1 初期カメラ

floor(2m四方)を含めた自動フィットだと対象（数cm）が小さく表示されるため、
本体サイズを基準にカメラを寄せる。

```python
_mdl.stat.center[:] = [0.0, -0.02, 0.03]
_mdl.stat.extent = 0.12
...
viewer.cam.lookat[:] = _mdl.stat.center
viewer.cam.distance  = _mdl.stat.extent * 2.5
viewer.cam.azimuth   = 180.0
viewer.cam.elevation = -30.0
```

新しいモデルでは、本体のだいたいの中心座標とサイズに合わせて `stat.center`/`stat.extent` を変更する。

### 9.2 表示用 qpos の反映

表示専用の `_dat` には物理演算（mj_step）を行わず、メインスレッド側 `libspikehat_sim` の
実シミュレーション状態（`hat.motor_get_position` 等）を `mj_kinematics`/`mj_comPos` で
反映するだけにする。`equality`制約はここでは評価されないため、`dome_joint` の角度も
`motor_joint` の角度から `polycoef` と同じ式で手動計算している。

```python
_motor_rad = math.radians(_hat.motor_get_position(0))
_dat.qpos[_motor_qadr] = _motor_rad
_dat.qpos[_dome_qadr]  = +_motor_rad / 3.0   # equalityのpolycoefと同じ式（符号は6.3参照）
```

新しいモデルでギア比が変わる場合は、ここの係数も `polycoef` と合わせて変更する。

### 9.3 Controlタブの操作をsimに転送

`press_ctrl`/`wall_x_ctrl`/`wall_y_ctrl` のように、ビューアのControlタブで人間が操作する
sim専用actuatorは、表示用 `_dat.ctrl` の値を毎フレーム `hat.sim_set_ctrl()` で
実シミュレーション側に転送し、結果のqposを表示用 `_dat.qpos` に反映する。

```python
_hat.sim_set_ctrl(_press_aid, float(_dat.ctrl[_press_aid]))
...
_dat.qpos[_press_qadr] = _hat.sim_get_qpos(_press_qadr)
```

壁のように複数のsim専用オブジェクトがある場合は、それぞれに同様の転送処理を追加する。
壁の移動はControlタブのスライダーに加え、キーボードコールバック（`key_callback`）からも
`_dat.ctrl` を直接書き換えることで操作できる（8.1参照）。

新しいモデルでsim専用の可動オブジェクトを追加する場合は、対応するactuator/jointの
id取得とこの転送処理を追加する。

### 9.4 実行速度（speed_scale）

`--speed` のデフォルトは **1.0（実時間）固定**。実機の動作時間とsimの動作時間を
直接比較できることが本プロジェクトの前提のため、デフォルト値は変更しない
（[[feedback_sonar_radar_realtime_sim]] 参照）。

---

## 10. libspikehat_simとの連携

### 10.1 概要

`libspikehat_sim` は `libspikehat` と同じAPIを持つMuJoCo版シミュレーターライブラリ。
`sonar_radar.xml` に上記のsite/sensor/actuatorを用意することで、
`raspi/sonar_radar.py` をそのままsimで動かせる。

```
実機（Raspberry Pi）:  raspi/sonar_radar.py → libspikehat     → Build HAT → SPIKE Prime
シム（Mac/Linux）   :  raspi/sonar_radar.py → libspikehat_sim → MuJoCo   → mujoco_model/sonar_radar.xml
```

### 10.2 sonar_radar_sim.py の仕組み

`sim/sonar_radar_sim.py` が `libspikehat_sim` の `SpikeHat` を `spikehat.SpikeHat` として
`sys.modules` に差し込み、`raspi/sonar_radar.py` を `importlib` で `__main__` として実行する
（ファイルのコピーではなく、`raspi/sonar_radar.py` を直接参照する）。

### 10.3 実行方法

```bash
cd sonar_radar/sim
python3 sonar_radar_sim.py                 # バッチ実行（実時間）
mjpython sonar_radar_sim.py --viewer       # ビューア付き（推奨、実時間）
```

`SPIKEHAT_SIM_XML` 環境変数でXMLパスを上書きできる（デフォルトは
`mujoco_model/sonar_radar.xml`）。

### 10.4 libspikehat_sim コンポーネントの include と可動部アタッチメント

#### MuJoCo `<include>` の制約

MuJoCo の `<include>` はXMLを**そのまま埋め込む**仕組みであり、
Unreal Engine や Unity のようなコンポーネントシステムとは異なる。
include 内で定義されたボディに、include を利用する側から**子ボディを追加することはできない**。

```xml
<body name="sensor_mount">
  <include file=".../force_sensor_body.xml"/>
  <!-- force_sensor_body.xml の中にある button body に、
       ここから子ボディを追加することはできない -->
</body>
```

#### include に適したコンポーネント

外部パーツが可動部に接続**しない**センサー類は include に適している。

| コンポーネント | include 適合 | 理由 |
|---|:---:|---|
| `distance_sensor_body.xml` | ✓ | ドームに取り付くだけ（可動部への接続なし） |
| `color_sensor_body.xml` | ✓ | 同上 |
| `force_sensor_body.xml` | △ | button body に press_block が接続する |

#### 可動部に外部パーツを接続する場合

press_block（シャフトを含む）のように、コンポーネント内の可動ボディに**剛体結合**する
外部パーツは、MuJoCo の `weld` equality + tight solref で近似する。

```xml
<!-- press_block を button に剛体結合（include の制約によりボディ階層での表現不可） -->
<equality>
  <weld body1="press_block" body2="button" solref="0.001 1"/>
</equality>
```

- `solref="0.001 1"` はデフォルト (`0.02 1`) より大幅に拘束を強め、残差を最小化する。
- 完全剛体ではないため、高速・高荷重の用途には注意が必要。

#### インライン展開による回避

どうしても親子階層で剛体結合する必要がある場合は、
include を使わず force_sensor_body.xml の内容を利用側XMLにインライン展開し、
press_block を button ボディの子として直接定義する。

```xml
<body name="button" pos="0 0 0">
  <joint name="button_slide" .../>
  <geom name="button_geom" .../>
  <body name="press_block" pos="...">   <!-- シャフトごと button の子として定義 -->
    <inertial .../>
    <geom name="press_geom" .../>
  </body>
</body>
```

この場合、force_sensor_body.xml の更新を手動でインライン版に反映する必要がある。

---

## 11. Studioモデルを変更したときの影響範囲

Studioモデルを変更した場合、変更の種類によって必要な作業が異なる。

### 11.1 影響範囲サマリー

| 変更の種類 | STL再生成 | XML変更 | アプリコード変更 |
|---|:---:|:---:|:---:|
| 固定部品の追加・削除 | 要 | 不要 | 不要 |
| 可動部の形状変更 | 要 | 要（pos） | 不要 |
| センサーの取り付け位置変更 | 要 | 要（site） | 不要 |
| ギア比の変更 | 要 | 要（equality） | 要（3箇所） |
| 新しい可動部の追加 | 要 | 要（joint等） | 要（sim.py） |

### 11.2 変更別の作業内容

#### 固定部品の追加・削除（例：台座に飾りパーツを追加）

1. `blender_export.py` を再実行してSTLを再生成
2. `radar_base_gray.stl` 等を使い続ける場合はXML変更不要（自動反映）
3. ただし `shared_offset`（全パーツのbboxから計算）が変わるため、
   赤・青マーカー等の位置関係がずれていないかビューアで確認する

#### 可動部の形状変更（例：ドーム形状を変更）

1. 対応するSTLを再生成（例：`radar_dome.stl`）
2. `body pos` の再調整が必要になる場合あり（重心・サイズの変化による）
3. センサーsite（`sonar_site`/`color_site`）の位置を再確認・微調整
   （[7. センサーsiteの配置調整](#7-センサーsiteの配置調整)の手順を再実施）

#### センサーの取り付け位置変更（例：距離センサーの位置を変える）

距離センサーは `libspikehat_sim` のコンポーネント（`include`）として組み込んでいる。
位置変更時は以下の手順を踏む：

1. `sonar_radar09.io` でセンサーの取り付け位置・向きを変更して保存
2. Blenderで `sonar_radar09.io` をインポートし、センサー (`37316c01.dat`) と
   基準ボディ (`bevel_gear_36` 等) の `matrix_world.translation` を取得
3. `pos = -(T_sensor - T_ref) * SCALE`（mj_x,mj_y符号反転、mj_z符号そのまま）を計算
4. `sonar_radar.xml` の `sonar_sensor` body の `pos` を更新
5. `euler` は `R_sensor_world @ Rx(-90°) @ Rz(180°)` をT_conj変換して求める
   （`studio_to_mujoco.md` 付属の変換スクリプトを参照）

カラーセンサーは現状インラインSTLのため従来通り `blender_export.py` を再実行。

#### ギア比の変更（例：12T-36T → 12T-24T）

影響が最も広い。以下の3箇所を**一致させる**こと。

| ファイル | 変更箇所 | 例（1:2に変更） |
|---|---|---|
| `mujoco_model/sonar_radar.xml` | `<equality>` の `polycoef` | `"0 -0.5 0 0 0"` |
| `sim/sonar_radar_sim.py` | `_viewer_loop` 内のドーム角度計算 | `-_motor_rad / 2.0` |
| `raspi/sonar_radar.py` | `GEAR_RATIO` 定数 | `GEAR_RATIO = 2` |

> **注意：** 回転方向の反転は `polycoef` の符号ではなく `dome_joint` の
> `axis` で調整する（[[feedback_mujoco_hinge_axis_vs_equality_sign]]）。

#### 新しい可動部の追加（例：新センサーを別軸で追加）

1. Studio側でサブモデルを追加・STLを再生成
2. XMLに `<body>` / `<joint>` / `<actuator>` / `<sensor>` を追加（§6〜§8参照）
3. `sim/sonar_radar_sim.py` の `_viewer_loop` に新しいjoint/actuatorの
   id取得とqpos反映処理を追加（§9.2〜9.3参照）
4. 新センサーを `raspi/sonar_radar.py` から使う場合はアプリコードも更新

---

## 12. 変更履歴

### v0.10.2（2026-07-06）

**フォースセンサーのストローク実測値への修正**

- ボタン突出量・ストロークを実測 8mm に修正（旧: 9.5mm）。
- `force_sensor_button.stl` を 8mm 突出量で再エクスポート。
- `button_slide` range 0〜0.008m、stiffness 1250 N/m（10N÷0.008m）に更新。
- `force_site` をボタン本体（可動）からセンサー本体ハウジング（固定）の Z=0.030m に移動。
  ストロークの 62% 押込み位置 = Hard-pressed 判定点。
- `press_block` pos Z を 0.0411 → 0.0405m に修正（ボタン位置変更に追従）。
- equality `solref="0.002 1"` でタイト拘束（重力残差 0.05mm 以下）。
- `libspikehat_sim` サブモジュールを b3591f1 に更新。

### v0.10.1（2026-07-05）

**ドキュメント画像を更新**

- `docs/sonar_radar_overview.png` を追加（`.jpg` から `.png` に変更）。
- `docs/sonar_radar_sim_snap.png` を更新（starter/フォースセンサーを含む最新外観）。
- `README.md` / `README_ja.md` の画像参照を `.jpg` から `.png` に変更。

### v0.10.0（2026-07-04）

**全センサーを libspikehat_sim の include コンポーネントに移行**

- 距離センサー・カラーセンサー・フォースセンサーの3つすべてを
  `libspikehat_sim/examples/components/` の `<include>` として組み込む方式に統一。
  各センサーの geom・joint・site は `sonar_radar.xml` のインライン定義から分離された。
- `blender_export.py` に `starter`（フォースセンサーユニット）の
  `sensor_mount` pos を自動計算するセクションを追加。
- カラーセンサーの `SENSOR_HOME_OFFSET` 符号修正（+5° → −5°）、
  `euler="-90 -185 0"` に更新。
- `button.stl` を `force_sensor_button.stl` にリネーム。
- `libspikehat_sim` の `force_sensor.io` の原点を
  センサー本体底面中心（LDraw 原点）に合わせた
  （`shared_offset=(0,0,0)` を確認済み）。
- `sonar_radar09.blend` を最新の Studio モデルから再生成。

### v0.9.0（2026-06-29）

**Hakoniwa asset 化・libspikehat_pdu 作成・結合テスト完了**
