#!/usr/bin/env python3
"""
sonar_radar.py - レーダースキャナー

ハードウェア構成:
  ポートA(0): Lアンギュラーモーター  - アーム回転
  ポートC(2): カラーセンサー          - 旋回端マーカー検出（赤=左端, 青=右端）
  ポートD(3): 距離センサー            - 障害物計測

座標系:
  0°=正面, 負方向=左端, 正方向=右端
  スキャンは SCAN_MIN_DEG〜SCAN_MAX_DEG を SCAN_STEP_DEG 刻み

使い方:
  python3 sonar_radar.py            # 標準スキャン（±65°）
  python3 sonar_radar.py --range 35 # 狭角スキャン（±35°）
"""

import sys
import time
import json
import argparse
sys.path.insert(0, '/home/kuboaki/projects/libspikehat/python')
from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE

# --- ハードウェア設定 ---
PORT_MOTOR    = 0   # ポートA
PORT_COLOR    = 2   # ポートC
PORT_DISTANCE = 3   # ポートD

# --- スキャン設定 ---
SCAN_STEP_DEG = 5       # 刻み幅（度）
DIST_MIN_MM   = 150     # 有効距離下限
DIST_MAX_MM   = 500     # 有効距離上限
DIST_INVALID  = 2000    # 測定不能値

# --- モーター設定 ---
CALIB_SPEED   = 25      # キャリブレーション時の速度
SCAN_SPEED    = 20      # スキャン時の速度
SETTLE_S      = 0.05    # モーター停止後の整定待ち（秒）

# --- カラー判定 ---
RED_SAT_MIN  = 40
RED_VAL_MIN  = 40
BLUE_HUE_LO  = 200
BLUE_HUE_HI  = 270
BLUE_SAT_MIN = 40
BLUE_VAL_MIN = 40


# --- モーター補助関数（libspikehat の既存APIを組み合わせ） ---

def motor_run_for_degrees(hat, port, degrees, speed):
    """指定角度だけ回転して停止する。libspikehat の汎用APIが追加されるまでの代替実装。"""
    if degrees == 0:
        return
    start = hat.motor_get_position(port)
    target = start + degrees
    actual_speed = speed if degrees > 0 else -speed
    hat.motor_start(port, actual_speed)
    while True:
        time.sleep(0.005)
        try:
            cur = hat.motor_get_position(port)
        except RuntimeError:
            continue
        if degrees > 0 and cur >= target:
            break
        if degrees < 0 and cur <= target:
            break
    hat.motor_stop(port)


# --- カラー判定 ---

def is_red(hue, sat, val):
    """赤マーカー判定（hueは色相環の両端付近）"""
    if sat < RED_SAT_MIN or val < RED_VAL_MIN:
        return False
    return hue >= 340 or hue <= 20


def is_blue(hue, sat, val):
    """青マーカー判定"""
    if sat < BLUE_SAT_MIN or val < BLUE_VAL_MIN:
        return False
    return BLUE_HUE_LO <= hue <= BLUE_HUE_HI


# --- 距離フィルタ ---

def filter_distance(mm):
    """有効距離ならmm値を、無効ならNoneを返す"""
    if mm == DIST_INVALID or mm < DIST_MIN_MM or mm > DIST_MAX_MM:
        return None
    return mm


# --- キャリブレーション ---

def calibrate(hat, scan_range):
    """
    反時計回りに回転して赤マーカー（左端）を探し、
    scan_range度 正方向に戻して0°（正面）を確立する。
    """
    print("キャリブレーション: 赤マーカーを探しています...", file=sys.stderr)
    hat.motor_start(PORT_MOTOR, -CALIB_SPEED)
    found = False
    for _ in range(200):    # 最大200×50ms = 10秒
        time.sleep(0.05)
        try:
            h, s, v = hat.color_read_hsv(PORT_COLOR)
            if is_red(h, s, v):
                found = True
                break
        except RuntimeError:
            pass
    hat.motor_stop(PORT_MOTOR)
    time.sleep(SETTLE_S)

    if not found:
        raise RuntimeError("赤マーカーが見つかりません。アームを正面付近に戻して再試行してください。")

    print(f"赤マーカー検出。{scan_range}度 正方向に移動して0°へ...", file=sys.stderr)
    motor_run_for_degrees(hat, PORT_MOTOR, scan_range, CALIB_SPEED)
    time.sleep(SETTLE_S)
    print("キャリブレーション完了 (現在位置 = 0°)", file=sys.stderr)


# --- スキャン ---

def do_scan(hat, scan_range):
    """
    -scan_range〜+scan_range を SCAN_STEP_DEG 刻みで計測する。
    戻り値: [{"angle": int, "distance_mm": int|None}, ...]
    """
    scan_min = -scan_range
    scan_max =  scan_range
    angles   = list(range(scan_min, scan_max + 1, SCAN_STEP_DEG))
    results  = []

    print(f"スキャン開始: {scan_min}°〜{scan_max}°, {SCAN_STEP_DEG}°刻み ({len(angles)}点)",
          file=sys.stderr)

    motor_run_for_degrees(hat, PORT_MOTOR, scan_min, SCAN_SPEED)
    time.sleep(SETTLE_S)
    current_deg = scan_min

    for angle in angles:
        move = angle - current_deg
        if move != 0:
            motor_run_for_degrees(hat, PORT_MOTOR, move, SCAN_SPEED)
            time.sleep(SETTLE_S)
            current_deg = angle

        # 3回計測して中央値を採用
        samples = []
        for _ in range(3):
            try:
                d = filter_distance(hat.distance_read(PORT_DISTANCE))
                if d is not None:
                    samples.append(d)
            except RuntimeError:
                pass
            time.sleep(0.02)

        dist = sorted(samples)[len(samples) // 2] if samples else None
        results.append({"angle": angle, "distance_mm": dist})
        label = f"{dist:5d} mm" if dist is not None else " null"
        print(f"  {angle:+4d}° -> {label}", file=sys.stderr)

    print("0°へ帰還...", file=sys.stderr)
    motor_run_for_degrees(hat, PORT_MOTOR, -current_deg, SCAN_SPEED)
    time.sleep(SETTLE_S)

    return results


# --- メイン ---

def main():
    parser = argparse.ArgumentParser(description="sonar_radar スキャナー")
    parser.add_argument("--range", type=int, default=65,
                        choices=[35, 65],
                        help="旋回範囲（片側の度数）: 35 または 65 (デフォルト: 65)")
    args = parser.parse_args()

    try:
        hat_instance = SpikeHat()
    except RuntimeError:
        print("エラー: Build HAT ファームウェアがロードされていません。", file=sys.stderr)
        print("run.sh を使うか、先に以下を実行してください:", file=sys.stderr)
        print("  python3 -c \"from buildhat import Motor; Motor('A')\"", file=sys.stderr)
        sys.exit(1)
    with hat_instance as hat:
        hat.port_config(PORT_MOTOR,    DEVICE_MOTOR_L)
        hat.port_config(PORT_COLOR,    DEVICE_COLOR)
        hat.port_config(PORT_DISTANCE, DEVICE_DISTANCE)
        time.sleep(1.0)

        calibrate(hat, args.range)
        results = do_scan(hat, args.range)

    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
