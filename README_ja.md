# sonar_radar

LEGO SPIKE Prime + Raspberry Pi Build HAT を使った2Dレーダースキャナー。
[libspikehat](https://github.com/kuboaki/libspikehat) 経由でデバイスを制御します。

![sonar_radar overview](docs/sonar_radar_overview.jpg)

## ハードウェア構成

| ポート | デバイス | 役割 |
|--------|---------|------|
| A (0) | SPIKE Prime Lアンギュラーモーター | アーム回転 |
| C (2) | SPIKE Prime カラーセンサー | 旋回端マーカー検出 |
| D (3) | SPIKE Prime 距離センサー | 障害物計測 |

### 旋回端マーカー

- **赤マーカー** — 左端（負方向）
- **青マーカー** — 右端（正方向）

±35° または ±65° の位置にマーカーを配置してスキャン範囲を決めます。

## スキャン仕様

| パラメータ | 値 |
|-----------|-----|
| スキャン範囲 | ±35° または ±65°（選択式）|
| ステップ角 | 3° |
| 有効距離 | 50〜300 mm |
| 原点（0°） | 正面中央 |

## 必要環境

- Raspberry Pi 4 + Build HAT
- **Raspberry Pi OS Bookworm (64bit)**
- [libspikehat](https://github.com/kuboaki/libspikehat) がビルド済みであること
- `python3-build-hat` インストール済み（`sudo apt install python3-build-hat`）

## 使い方

```bash
bash run.sh            # 標準スキャン（±65°）
bash run.sh --range 35 # 狭角スキャン（±35°）
```

`run.sh` はスキャン前に Build HAT ファームウェアをロードします。
`sonar_radar.py` を直接呼び出さず、必ず `run.sh` 経由で実行してください。

## 出力形式

標準出力に JSON 配列、ログは標準エラー出力に出力されます。

```json
[
  {"angle": -65, "distance_mm": 312},
  {"angle": -60, "distance_mm": 298},
  {"angle": -55, "distance_mm": null},
  ...
]
```

有効距離範囲外の場合は `distance_mm` が `null` になります。

## キャリブレーション

起動時に反時計回りでカラーセンサーが赤マーカー（左端）を検出するまで回転し、
その後 0°（正面）へ移動します。事前の手動位置合わせは不要です。

## プロジェクト構成

```
sonar_radar/
├── sonar_radar.py   メインスキャナースクリプト
├── run.sh           起動スクリプト（ファームウェアロード＋スキャン実行）
├── docs/            ドキュメント用画像
└── model/           LEGO Studioモデルファイル
```

## ライセンス

MIT License
