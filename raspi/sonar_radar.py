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
import os
import json
import argparse
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'libspikehat', 'python'))

from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE

# --- ハードウェア設定 ---
PORT_MOTOR    = 0   # ポートA
PORT_COLOR    = 2   # ポートC
PORT_DISTANCE = 3   # ポートD

# --- スキャン設定 ---
SCAN_STEP_DEG = 3       # 刻み幅（度）
DIST_MIN_MM   = 50      # 有効距離下限
DIST_MAX_MM   = 300     # 有効距離上限
DIST_INVALID  = 2000    # 測定不能値
DIST_OFFSET_MM = 25     # センサー面が旋回軸より前方にある分の距離補正値（mm）

# --- モーター設定 ---
CALIB_SPEED       = 12      # キャリブレーション時の速度
SCAN_SPEED        = 7       # ステップスキャン時の速度
CONTINUOUS_SPEED  = 7       # 連続スキャン時の速度
SETTLE_S          = 0.15    # モーター停止後の整定待ち（秒）
SAMPLE_INTERVAL_S = 0.10    # 連続スキャンのサンプリング間隔（距離センサー更新周期）

# sonar_dome はベベルギア(12T-36T)の噛み合わせ上、Lモーターの機械的0位置で
# 正面から5度ズレた状態になる。この補正値を加えた位置を論理的な0°（正面）とする。
SENSOR_HOME_OFFSET = 5

# --- カラー判定 ---
BLUE_HUE_LO  = 210
BLUE_HUE_HI  = 270
BLUE_SAT_MIN = 580
BLUE_VAL_MIN = 100


# --- カラー判定 ---

def is_blue(hue, sat, val):
    """青マーカー判定"""
    if sat < BLUE_SAT_MIN or val < BLUE_VAL_MIN:
        return False
    return BLUE_HUE_LO <= hue <= BLUE_HUE_HI


# --- 距離フィルタ ---

def filter_distance(mm):
    """センサー生値を受け取り、旋回軸基準の有効距離を返す。範囲外はNone。"""
    if mm == DIST_INVALID:
        return None
    corrected = mm + DIST_OFFSET_MM  # センサー面オフセット分を加算して旋回軸基準の距離に変換
    if corrected < DIST_MIN_MM or corrected > DIST_MAX_MM:
        return None
    return corrected


# --- キャリブレーション ---

def calibrate(hat):
    """
    Lモーターを機械的0位置へ移動し、そこから SENSOR_HOME_OFFSET 度だけ
    旋回してドームを正面に向ける。この位置を0°（正面）とする。
    """
    print("キャリブレーション: 機械的0位置へ移動...", file=sys.stderr)
    hat.motor_run_to_position(PORT_MOTOR, 0, CALIB_SPEED)
    hat.sleep(SETTLE_S)

    print(f"SENSOR_HOME_OFFSET({SENSOR_HOME_OFFSET}度)分を補正...", file=sys.stderr)
    hat.motor_run_to_position(PORT_MOTOR, SENSOR_HOME_OFFSET, CALIB_SPEED)
    hat.sleep(SETTLE_S)

    zero_pos = hat.motor_get_position(PORT_MOTOR)
    print(f"キャリブレーション完了 (現在位置 = 0°, encoder={zero_pos})", file=sys.stderr)
    return zero_pos


# --- スキャン ---

def do_scan(hat, scan_range, zero_pos):
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

    hat.motor_run_for_degrees(PORT_MOTOR, scan_min, SCAN_SPEED)
    hat.sleep(SETTLE_S)

    for angle in angles:
        # 実際の現在位置から移動量を計算して累積誤差を補正
        try:
            actual_now = hat.motor_get_position(PORT_MOTOR) - zero_pos
        except RuntimeError:
            actual_now = angle
        move = angle - actual_now
        if abs(move) > 1:
            hat.motor_run_for_degrees(PORT_MOTOR, move, SCAN_SPEED)
            hat.sleep(SETTLE_S)

            # 正方向移動後に青マーカー（右端）を検出したらスキャン終了
            if move > 0:
                try:
                    h, s, v = hat.color_read_hsv(PORT_COLOR)
                    if is_blue(h, s, v):
                        print("青マーカー検出: スキャン右端に達しました", file=sys.stderr)
                        break
                except RuntimeError:
                    pass

        # 3回計測して中央値を採用
        samples = []
        for _ in range(3):
            try:
                d = filter_distance(hat.distance_read(PORT_DISTANCE))
                if d is not None:
                    samples.append(d)
            except RuntimeError:
                pass
            hat.sleep(0.02)

        dist = sorted(samples)[len(samples) // 2] if samples else None
        try:
            actual_angle = hat.motor_get_position(PORT_MOTOR) - zero_pos
        except RuntimeError:
            actual_angle = angle
        results.append({"angle": actual_angle, "distance_mm": dist})
        label = f"{dist:5d} mm" if dist is not None else " null"
        print(f"  cmd:{angle:+4d}° actual:{actual_angle:+4d}° -> {label}", file=sys.stderr)

    # 実際の位置から0°へ帰還
    print("0°へ帰還...", file=sys.stderr)
    try:
        actual_now = hat.motor_get_position(PORT_MOTOR) - zero_pos
    except RuntimeError:
        actual_now = 0
    hat.motor_run_for_degrees(PORT_MOTOR, -actual_now, SCAN_SPEED)
    hat.sleep(SETTLE_S)

    return results


# --- 連続スキャン ---

def do_continuous_scan(hat, scan_range, zero_pos):
    """
    -scan_range から +scan_range まで連続回転しながら
    SAMPLE_INTERVAL_S ごとに角度と距離を記録する。
    戻り値: [{"angle": int, "distance_mm": int|None}, ...]
    """
    scan_min = -scan_range
    scan_max =  scan_range
    results  = []

    print(f"連続スキャン開始: {scan_min}°〜{scan_max}°, "
          f"速度={CONTINUOUS_SPEED}, 間隔={SAMPLE_INTERVAL_S*1000:.0f}ms",
          file=sys.stderr)

    # スキャン開始位置へ移動
    hat.motor_run_for_degrees(PORT_MOTOR, scan_min, CALIB_SPEED)
    hat.sleep(SETTLE_S)

    # 連続回転しながらサンプリング
    hat.motor_start(PORT_MOTOR, CONTINUOUS_SPEED)
    while True:
        hat.sleep(SAMPLE_INTERVAL_S)

        try:
            actual_angle = hat.motor_get_position(PORT_MOTOR) - zero_pos
        except RuntimeError:
            continue

        try:
            dist = filter_distance(hat.distance_read(PORT_DISTANCE))
        except RuntimeError:
            dist = None

        results.append({"angle": actual_angle, "distance_mm": dist})
        label = f"{dist:5d} mm" if dist is not None else " null"
        print(f"  {actual_angle:+4d}° -> {label}", file=sys.stderr)

        # 青マーカー検出 or 目標角度到達でスキャン終了
        if actual_angle >= scan_max:
            break
        try:
            h, s, v = hat.color_read_hsv(PORT_COLOR)
            if is_blue(h, s, v):
                print("青マーカー検出: スキャン右端に達しました", file=sys.stderr)
                break
        except RuntimeError:
            pass

    hat.motor_stop(PORT_MOTOR)
    hat.sleep(SETTLE_S)

    # 0°へ帰還
    print("0°へ帰還...", file=sys.stderr)
    try:
        actual_now = hat.motor_get_position(PORT_MOTOR) - zero_pos
    except RuntimeError:
        actual_now = 0
    hat.motor_run_for_degrees(PORT_MOTOR, -actual_now, CALIB_SPEED)
    hat.sleep(SETTLE_S)

    return results


# --- メイン ---

def main():
    parser = argparse.ArgumentParser(description="sonar_radar スキャナー")
    parser.add_argument("--range", type=int, default=65,
                        choices=[35, 65],
                        help="旋回範囲（片側の度数）: 35 または 65 (デフォルト: 65)")
    parser.add_argument("--mode", choices=["step", "continuous"], default="step",
                        help="スキャンモード: step=停止計測, continuous=連続計測 (デフォルト: step)")
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
        hat.sleep(1.0)

        zero_pos = calibrate(hat)
        if args.mode == "continuous":
            results = do_continuous_scan(hat, args.range, zero_pos)
        else:
            results = do_scan(hat, args.range, zero_pos)

    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
