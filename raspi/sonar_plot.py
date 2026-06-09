#!/usr/bin/env python3
"""
sonar_plot.py - sonar_radar スキャン結果を扇形グラフで表示する

使い方:
  python3 sonar_plot.py scan.json       # ファイルから読み込み
  bash run.sh | python3 sonar_plot.py   # パイプで受け取り
"""

import sys
import json
import math
import argparse
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

DIST_MAX_MM = 300      # 表示距離上限 (mm)
MARGIN_DEG  = 8        # 表示余白（度）
OUTPUT_FILE = "scan_result.png"


def to_xy(angle_deg, dist_mm):
    """角度(0°=上、正=右)と距離をXY座標に変換"""
    r = math.radians(angle_deg)
    return dist_mm * math.sin(r), dist_mm * math.cos(r)


def plot_scan(results, output, title):
    all_angles = [r["angle"] for r in results]
    scan_min = min(all_angles) - MARGIN_DEG
    scan_max = max(all_angles) + MARGIN_DEG

    fig, ax = plt.subplots(figsize=(6, 4))
    ax.set_aspect('equal')

    # --- 背景グリッド ---
    # 同心円（距離リング）
    for d in [100, 200, 300]:
        circle = plt.Circle((0, 0), d, fill=False,
                             color='lightgray', linestyle='--', linewidth=0.8)
        ax.add_patch(circle)
        ax.text(4, d + 5, f'{d}', fontsize=7, color='gray')

    # 放射線（角度ガイド）
    for a in range(int(math.ceil(scan_min/15))*15,
                   int(math.floor(scan_max/15))*15 + 1, 15):
        x, y = to_xy(a, DIST_MAX_MM)
        ax.plot([0, x], [0, y], color='lightgray', linestyle='--',
                linewidth=0.8)
        lx, ly = to_xy(a, DIST_MAX_MM + 15)
        label = f'{a:+d}°' if a != 0 else '0°'
        ax.text(lx, ly, label, fontsize=8, ha='center', va='center',
                color='dimgray')

    # スキャン範囲の扇形の外枠
    arc_angles = np.linspace(math.radians(scan_min),
                             math.radians(scan_max), 100)
    arc_x = DIST_MAX_MM * np.sin(arc_angles)
    arc_y = DIST_MAX_MM * np.cos(arc_angles)
    ax.plot(arc_x, arc_y, color='lightgray', linewidth=1.0)
    ax.plot([0, arc_x[0]], [0, arc_y[0]], color='lightgray', linewidth=1.0)
    ax.plot([0, arc_x[-1]], [0, arc_y[-1]], color='lightgray', linewidth=1.0)

    # --- データのプロット ---
    # 検出なし
    null_xs, null_ys = [], []
    for r in results:
        if r["distance_mm"] is None or r["distance_mm"] > DIST_MAX_MM:
            x, y = to_xy(r["angle"], DIST_MAX_MM)
            null_xs.append(x)
            null_ys.append(y)
    if null_xs:
        ax.scatter(null_xs, null_ys, c='lightgray', s=12, zorder=2,
                   label='no detection')

    # 検出あり
    det = [(r["angle"], r["distance_mm"]) for r in results
           if r["distance_mm"] is not None and r["distance_mm"] <= DIST_MAX_MM]
    if det:
        det.sort()
        xs = [to_xy(a, d)[0] for a, d in det]
        ys = [to_xy(a, d)[1] for a, d in det]
        ax.plot(xs, ys, color='tomato', linewidth=1.5, alpha=0.7, zorder=3)
        ax.scatter(xs, ys, c='red', s=40, zorder=4, label='detected')

    # --- 軸設定 ---
    lim = DIST_MAX_MM + 30
    edge_x = lim * math.sin(math.radians(max(abs(scan_min), abs(scan_max))))
    ax.set_xlim(-edge_x - 20, edge_x + 20)
    ax.set_ylim(-20, lim + 20)
    ax.axis('off')
    ax.set_title(title, fontsize=11, pad=10)
    ax.legend(loc='lower right', fontsize=8)

    plt.tight_layout()
    plt.savefig(output, dpi=150, bbox_inches='tight')
    print(f"saved: {output}", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="sonar_radar 結果を扇形グラフで出力")
    parser.add_argument("input", nargs="?", help="JSON ファイル (省略時は標準入力)")
    parser.add_argument("-o", "--output", default=OUTPUT_FILE,
                        help=f"出力ファイル (デフォルト: {OUTPUT_FILE})")
    parser.add_argument("--title", default="sonar_radar scan")
    args = parser.parse_args()

    if args.input:
        with open(args.input) as f:
            results = json.load(f)
    else:
        results = json.load(sys.stdin)

    if not results:
        print("エラー: データが空です", file=sys.stderr)
        sys.exit(1)

    plot_scan(results, args.output, args.title)


if __name__ == "__main__":
    main()
