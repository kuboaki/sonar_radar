# スキャン結果の可視化手順

実機・SIMのスキャン結果を取得し、`raspi/sonar_plot.py` で可視化するまでの手順。
`docs/scan_real.json`/`docs/scan_real_example.png`、
`docs/scan_sim.json`/`docs/scan_sim_example.png` はこの手順で作成した例。

## 1. 実機データの取得

Raspberry Pi上で `raspi/run.sh` を実行し、標準出力のJSONをファイルに保存する。

```bash
cd ~/projects/sonar_radar/raspi
bash run.sh > scan_real.json
```

フォースセンサー（終了スイッチ）を押すとスキャンが終了し、JSONが出力される。
作成した `scan_real.json` を、Mac側のリポジトリの `docs/` にコピーする（`scp` 等）。

## 2. SIMデータの取得

### 2.1 注意点：バッチ実行はスキャンが終了しない

`sim/sonar_radar_sim.py` をビューアなし（`python3`）で実行すると、
フォースセンサー（`press_body`）を押す手段がないため、`do_continuous_scan` の
終了条件が満たされず、標準出力にJSONが出力されない（プロセスを止めるまで動き続ける）。

### 2.2 手順：標準エラーのログから再構成する

実用上は、標準エラー出力（`[ xx.xxs] motor:... dome:... -> ...`形式のログ）を
タイムアウト等で適当な時間だけ取得し、そこから `{"angle":..., "dome_angle":...,
"distance_mm":...}` のJSON配列を再構成すれば `sonar_plot.py` にそのまま使える。

```bash
cd sim
timeout 45 python3 sonar_radar_sim.py > /dev/null 2>sim_scan.log
```

```python
# sim_scan.log -> scan_sim.json
import re, json

results = []
pat = re.compile(r"motor:\s*([+-]?\d+)°\s*dome:\s*([+-]?\d+\.\d+)°\s*->\s*(null|\d+)\s*(?:mm)?")
for line in open("sim_scan.log"):
    m = pat.search(line)
    if not m:
        continue
    angle = int(m.group(1))
    dome = float(m.group(2))
    dist = None if m.group(3) == "null" else int(m.group(3))
    results.append({"angle": angle, "dome_angle": dome, "distance_mm": dist})

json.dump(results, open("scan_sim.json", "w"))
```

> ビューア（`mjpython sonar_radar_sim.py --viewer`）を使い、Controlタブの
> `press_ctrl` を操作してスキャンを正規に終了させれば、標準出力のJSONを
> そのまま使うこともできる。

## 3. 可視化

```bash
python3 raspi/sonar_plot.py docs/scan_real.json -o docs/scan_real_example.png --title "実機スキャン結果"
python3 raspi/sonar_plot.py docs/scan_sim.json  -o docs/scan_sim_example.png  --title "SIMスキャン結果"
```

`sonar_plot.py` は `matplotlib`/`numpy` に依存する（`pip install matplotlib`）。
日本語フォントは `Noto Sans CJK JP`（Linux）/ `Hiragino Sans`（macOS）/
`IPAGothic` のいずれかが必要。
