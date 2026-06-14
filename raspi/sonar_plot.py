#!/usr/bin/env python3
"""
sonar_plot.py - sonar_radar スキャン結果を可視化する

sonar_radar.py の出力（"angle"=モーター角度, "dome_angle"=dome角度,
"distance_mm"=距離）を読み込み、以下を表示する。

  左: dome角度 vs 距離 の扇形プロット（往復パスを重ね描き）
  右: dome角度 vs 距離 の折れ線プロット（往復パスを重ね描き）

データは一定角度間隔ではないため、点列をそのまま使用する。
旋回方向が反転するたびに新しい「パス」として区切り、
各パスを別の色で重ねて表示することで往復差を比較できる。

使い方:
  python3 sonar_plot.py scan.json
  python3 sonar_radar.py > scan.json && python3 sonar_plot.py scan.json
"""

import sys
import json
import math
import argparse
import matplotlib
matplotlib.use('Agg')
matplotlib.rcParams['font.family'] = ['Noto Sans CJK JP', 'Hiragino Sans', 'IPAGothic']
import matplotlib.pyplot as plt
import numpy as np

DIST_MAX_MM = 300      # 表示距離上限 (mm)
MARGIN_DEG  = 8         # 表示余白（度）
OUTPUT_FILE = "scan_result.png"


def to_xy(angle_deg, dist_mm):
    """角度(0°=上、正=右)と距離をXY座標に変換"""
    r = math.radians(angle_deg)
    return dist_mm * math.sin(r), dist_mm * math.cos(r)


def split_passes(results):
    """dome_angle の旋回方向が反転するごとにパスへ分割する"""
    passes = []
    current = []
    prev_angle = None
    prev_dir = 0

    for r in results:
        angle = r.get("dome_angle")
        if angle is None:
            continue

        if prev_angle is not None:
            diff = angle - prev_angle
            if diff != 0:
                direction = 1 if diff > 0 else -1
                if prev_dir != 0 and direction != prev_dir:
                    if len(current) >= 2:
                        passes.append(current)
                    current = [current[-1]] if current else []
                prev_dir = direction

        current.append(r)
        prev_angle = angle

    if len(current) >= 2:
        passes.append(current)

    return passes


def plot_fan(ax, passes, scan_min, scan_max, colors):
    # --- 背景グリッド ---
    for d in [100, 200, 300]:
        circle = plt.Circle((0, 0), d, fill=False,
                             color='lightgray', linestyle='--', linewidth=0.8)
        ax.add_patch(circle)
        ax.text(4, d + 5, f'{d}', fontsize=7, color='gray')

    for a in range(int(math.ceil(scan_min/15))*15,
                   int(math.floor(scan_max/15))*15 + 1, 15):
        x, y = to_xy(a, DIST_MAX_MM)
        ax.plot([0, x], [0, y], color='lightgray', linestyle='--',
                linewidth=0.8)
        lx, ly = to_xy(a, DIST_MAX_MM + 15)
        label = f'{a:+d}°' if a != 0 else '0°'
        ax.text(lx, ly, label, fontsize=8, ha='center', va='center',
                color='dimgray')

    arc_angles = np.linspace(math.radians(scan_min),
                             math.radians(scan_max), 100)
    arc_x = DIST_MAX_MM * np.sin(arc_angles)
    arc_y = DIST_MAX_MM * np.cos(arc_angles)
    ax.plot(arc_x, arc_y, color='lightgray', linewidth=1.0)
    ax.plot([0, arc_x[0]], [0, arc_y[0]], color='lightgray', linewidth=1.0)
    ax.plot([0, arc_x[-1]], [0, arc_y[-1]], color='lightgray', linewidth=1.0)

    # --- パスごとに重ね描き ---
    for i, p in enumerate(passes):
        det = [(r["dome_angle"], r["distance_mm"]) for r in p
               if r["distance_mm"] is not None and r["distance_mm"] <= DIST_MAX_MM]
        if not det:
            continue
        xs = [to_xy(a, d)[0] for a, d in det]
        ys = [to_xy(a, d)[1] for a, d in det]
        c = colors[i % len(colors)]
        ax.plot(xs, ys, color=c, linewidth=1.2, alpha=0.7, zorder=3)
        ax.scatter(xs, ys, c=[c], s=18, zorder=4, label=f'pass {i+1}')

    lim = DIST_MAX_MM + 30
    edge_x = lim * math.sin(math.radians(max(abs(scan_min), abs(scan_max))))
    ax.set_aspect('equal')
    ax.set_xlim(-edge_x - 20, edge_x + 20)
    ax.set_ylim(-20, lim + 20)
    ax.axis('off')
    ax.set_title('dome角度 vs 距離（扇形）', fontsize=11, pad=10)
    ax.legend(loc='lower right', fontsize=8)


def plot_line(ax, passes, colors):
    for i, p in enumerate(passes):
        det = [(r["dome_angle"], r["distance_mm"]) for r in p
               if r["distance_mm"] is not None]
        if not det:
            continue
        xs = [a for a, d in det]
        ys = [d for a, d in det]
        c = colors[i % len(colors)]
        ax.plot(xs, ys, color=c, marker='o', markersize=3,
                linewidth=1.0, alpha=0.7, label=f'pass {i+1}')

    ax.set_xlabel('dome角度 [deg]')
    ax.set_ylabel('距離 [mm]')
    ax.set_title('dome角度 vs 距離（折れ線）', fontsize=11, pad=10)
    ax.grid(True, linestyle='--', linewidth=0.5, alpha=0.6)
    ax.legend(fontsize=8)


def plot_scan(results, output, title):
    passes = split_passes(results)
    if not passes:
        print("エラー: 有効な角度データがありません", file=sys.stderr)
        sys.exit(1)

    all_angles = [r["dome_angle"] for r in results if r.get("dome_angle") is not None]
    scan_min = min(all_angles) - MARGIN_DEG
    scan_max = max(all_angles) + MARGIN_DEG

    colors = ['tomato', 'royalblue', 'seagreen', 'darkorange',
              'mediumpurple', 'goldenrod', 'teal', 'crimson']

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
    plot_fan(ax1, passes, scan_min, scan_max, colors)
    plot_line(ax2, passes, colors)

    fig.suptitle(title, fontsize=12)
    plt.tight_layout()
    plt.savefig(output, dpi=150, bbox_inches='tight')
    print(f"saved: {output} (passes={len(passes)})", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(description="sonar_radar 結果を可視化")
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
