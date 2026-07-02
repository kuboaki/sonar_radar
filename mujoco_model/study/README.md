# study/ — センサーコンポーネント include 方式への移行検討資料

このディレクトリには、distance_sensor および color_sensor を
libspikehat_sim の `<include>` コンポーネント方式に移行するにあたり
作成した検討・検証用の MuJoCo モデルを保存しています。

同様の作業を新しいプロジェクトで行う際の参考にしてください。

---

## 背景と問題意識

### インライン版の問題

移行前は、センサーの `<inertial>` / `<geom>` / `<site>` を
sonar_radar.xml に直接書く「インライン版」を使っていた。

インライン版の欠点:
- 複数モデルで同じセンサーを使う場合に XML が重複する
- センサー仕様が変わったときに全モデルを手修正する必要がある
- STL の座標系規約がモデルごとに異なり、位置合わせが困難になる

### include 方式の採用

libspikehat_sim の `examples/components/` 以下に
`distance_sensor_body.xml` / `color_sensor_body.xml` を置き、
利用側モデルは `<include file="..."/>` で取り込む方式に統一した。

---

## MuJoCo `<include>` の重要な挙動

MuJoCo の `<include>` は **ルート `<body>` タグを除去** し、
その子要素（`<inertial>` / `<geom>` / `<site>`）のみを
親 body に挿入する。

```xml
<!-- color_sensor_body.xml の内容 -->
<body name="color_sensor" euler="-90 0 0">   ← このタグは除去される
  <inertial .../>
  <geom .../>
  <site .../>
</body>
```

```xml
<!-- 利用側モデル -->
<body name="color_sensor" pos="X Y Z" euler="-90 0 0">  ← euler はここに書く
  <include file=".../color_sensor_body.xml"/>
</body>
```

**euler や joint は必ず外側の body に書くこと。**
コンポーネントファイル内のルート body の euler は無視される。

---

## STL 座標系規約: pivot_origin 方式

distance_sensor / color_sensor ともに `pivot_origin` 方式を採用している。

- STL の原点 = LDraw パーツの基準点（ピボット）
- Blender でインポートする際は `.io` ではなく `.ldr` を直接インポートする
  （Studio の床スナップで Z=19.5 LDU がつくため）
- blender_export_*.py の `center_mode="pivot_origin"` で生成する

`bottom_z`（STL 底面 Z=0）方式も試みたが、
モデルによってパーツの底面定義が異なり、汎用コンポーネントに向かないため廃止。

---

## ドームの5°傾き補正

sonar_radar のドームは、LDraw ジオメトリレベルで5°傾いている
（LDraw 上のボディ配置ではなくメッシュ形状に焼き込まれている）。

そのため、ドーム上のすべてのセンサーの euler に
**親フレーム（gear36）での Rz(+5°) 前置乗算** が必要になる。

### distance_sensor の場合

`blender_export.py` の `compute_local_body_euler()` が
ドームの実姿勢を含む world 回転を計算するため、
Blender 上で5°傾きが自動的に捉えられた。

結果: `euler="0 0 -175"` （-180° + 5° の補正）

### color_sensor の場合

Blender 上での計算結果は `euler="-90 -180 0"` （5°成分なし）だった。
これはカラーセンサーの LDraw 配置がドーム傾きを陽に反映していないため。

手動で Rz(+5°) を親フレームで前置乗算して euler を再計算した:

```
R_corrected = Rz(+5°) × Rx(-90°) × Ry(-180°)
→ euler="90 5 180"  （XYZ 分解結果）
```

**注意**: Rz(+5°) を XYZ euler の Rz 成分（第3引数）に加算するだけでは誤り。
intrinsic（ボディフレーム）回転になってしまい、親フレームでの補正にならない。

---

## 検証用モデルの説明

### test_distance_include_rotation.xml

distance_sensor を `<include>` 方式で単体検証するモデル。
gear36 相当のダミー body を原点に置き、その子として
sonar_sensor を配置してビューアで向きを確認するために使った。

pos / euler は最終的に sonar_radar.xml に採用した値と異なる場合がある
（試行錯誤の途中段階のものが残っている）。

### test_old_vs_new_sensor.xml

距離センサーの「インライン版（旧）」と「include 版（新）」を
ドームメッシュと一緒に同じシーンに並べて位置を比較したモデル。

- 旧インライン版: 緑色（rgba 0.2 0.9 0.2）
- 新 include 版:  赤色（include 版は色変更できないため構造で識別）

include 版の pos をインライン版に合わせ込む作業に使った。

---

## 今後の新プロジェクトへの適用手順

1. Studio でロボットモデルにセンサーを配置した `.io` を作成
2. Blender で `.ldr`（Studio 床スナップ対策、手動で Z=0 維持）をインポート
3. `blender_export_*.py` を実行して STL と pos/euler を取得
4. 利用側モデルで outer body に pos/euler を設定し `<include>` を挿入
5. ドーム等の傾きがある場合は親フレームでの回転補正を手計算で追加
6. ビューアで視覚確認し、site（黄色球）が検出面から見えることを確認

詳細は `libspikehat_sim/docs/` も参照すること。
