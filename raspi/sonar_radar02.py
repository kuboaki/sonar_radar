#!/usr/bin/env python3
"""
sonar_radar02.py - レーダースキャナー（PWM旋回・マーカー反転版）

ハードウェア構成（減速ギア経由でドームを旋回）:
  ポートA(0): Lアンギュラーモーター  - ドーム旋回（ギア減速 1:3、回転方向反転）
  ポートB(1): フォースセンサー       - 終了スイッチ
  ポートC(2): カラーセンサー         - 旋回端マーカー検出（赤=左端, 青=右端）
  ポートD(3): 距離センサー           - 障害物計測

動作:
  1. 0位置へ旋回（return_to_origin）
  2. PWMで旋回し、赤・青マーカーを検出したら旋回方向を反転
  3. インターバルごとに角度と距離を記録
  4. フォースセンサーが押されるまで 2,3 を繰り返し
  5. 押されたら停止し、結果をJSONで出力
"""

import sys
import json
sys.path.insert(0, '/home/kuboaki/projects/libspikehat/python')

from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_FORCE, DEVICE_COLOR, DEVICE_DISTANCE

# --- ハードウェア設定 ---
PORT_MOTOR    = 0   # ポートA
PORT_FORCE    = 1   # ポートB
PORT_COLOR    = 2   # ポートC
PORT_DISTANCE = 3   # ポートD

# --- 距離設定 ---
DIST_MIN_MM    = 50      # 有効距離下限
DIST_MAX_MM    = 300     # 有効距離上限
DIST_INVALID   = 2000    # 測定不能値
DIST_OFFSET_MM = 25      # センサー面が旋回軸より前方にある分の距離補正値（mm）

# --- モーター設定（PWM） ---
SCAN_PWM          = 0.1    # 旋回速度（デューティ比）
ALIGN_SPEED       = 10     # return_to_origin 時の速度
SAMPLE_INTERVAL_S = 0.10   # サンプリング間隔（秒）

# --- ギア比（モーター:dome = 1:3、回転方向反転） ---
GEAR_RATIO = 3

# --- カラー判定 ---
RED_SAT_MIN  = 40
RED_VAL_MIN  = 40
BLUE_HUE_LO  = 210
BLUE_HUE_HI  = 270
BLUE_SAT_MIN = 580
BLUE_VAL_MIN = 100


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
    """センサー生値を受け取り、旋回軸基準の有効距離を返す。範囲外はNone。"""
    if mm == DIST_INVALID:
        return None
    corrected = mm + DIST_OFFSET_MM
    if corrected < DIST_MIN_MM or corrected > DIST_MAX_MM:
        return None
    return corrected


# --- 0位置への復帰 ---
def return_to_origin(hat, port):
    """現在位置から原点(0度)へ戻す"""
    cur_pos = hat.motor_get_position(port)
    if cur_pos == 0:
        return

    print(f"0位置へ復帰: 現在位置 {cur_pos} 度 -> 0 度", file=sys.stderr)
    hat.motor_run_for_degrees(port, -cur_pos, ALIGN_SPEED)

    dur = (abs(cur_pos) / 360.0) / (ALIGN_SPEED * 0.05)
    if dur < 0.5:
        dur = 0.5
    hat.sleep(dur + 0.5)


# --- 連続スキャン ---
def do_continuous_scan(hat):
    """
    PWMで旋回しながら SAMPLE_INTERVAL_S ごとに角度と距離を記録する。
    赤・青マーカーを検出するたびに旋回方向を反転し、
    フォースセンサーが押されたら終了する。
    戻り値: [{"angle": int, "distance_mm": int|None}, ...]
    """
    results  = []
    scan_pwm = SCAN_PWM
    on_marker = False

    print(f"連続スキャン開始: 速度(PWM)={scan_pwm}, 間隔={SAMPLE_INTERVAL_S*1000:.0f}ms",
          file=sys.stderr)

    hat.motor_pwm(PORT_MOTOR, scan_pwm)

    while True:
        # マーカー検出でスキャン方向を反転（エッジ検出）
        try:
            h, s, v = hat.color_read_hsv(PORT_COLOR)
            marker = is_red(h, s, v) or is_blue(h, s, v)
            if marker and not on_marker:
                color_name = "赤" if is_red(h, s, v) else "青"
                print(f"{color_name}マーカー検出: 反転します", file=sys.stderr)
                scan_pwm = -scan_pwm
                hat.motor_pwm(PORT_MOTOR, scan_pwm)
            on_marker = marker
        except RuntimeError:
            pass

        # 角度・距離を記録
        try:
            angle = hat.motor_get_position(PORT_MOTOR)
            dome_angle = -angle / GEAR_RATIO
        except RuntimeError:
            angle = None
            dome_angle = None

        try:
            dist = filter_distance(hat.distance_read(PORT_DISTANCE))
        except RuntimeError:
            dist = None

        results.append({"angle": angle, "dome_angle": dome_angle, "distance_mm": dist})
        label = f"{dist:5d} mm" if dist is not None else " null"
        angle_label = f"{angle:+4d}" if angle is not None else "  --"
        dome_label = f"{dome_angle:+6.1f}" if dome_angle is not None else "    --"
        print(f"  motor:{angle_label}° dome:{dome_label}° -> {label}", file=sys.stderr)

        # フォースセンサー押下で終了
        try:
            if hat.force_is_pressed(PORT_FORCE):
                print("フォースセンサー押下: スキャン終了", file=sys.stderr)
                break
        except RuntimeError:
            pass

        hat.sleep(SAMPLE_INTERVAL_S)

    hat.motor_stop(PORT_MOTOR)
    hat.sleep(0.5)

    return results


# --- メイン ---
def main():
    try:
        hat_instance = SpikeHat()
    except RuntimeError:
        print("エラー: Build HAT ファームウェアがロードされていません。", file=sys.stderr)
        print("run.sh を使うか、先に以下を実行してください:", file=sys.stderr)
        print("  python3 -c \"from buildhat import Motor; Motor('A')\"", file=sys.stderr)
        sys.exit(1)

    with hat_instance as hat:
        hat.port_config(PORT_MOTOR,    DEVICE_MOTOR_L)
        hat.port_config(PORT_FORCE,    DEVICE_FORCE)
        hat.port_config(PORT_COLOR,    DEVICE_COLOR)
        hat.port_config(PORT_DISTANCE, DEVICE_DISTANCE)
        hat.sleep(1.0)

        return_to_origin(hat, PORT_MOTOR)

        results = do_continuous_scan(hat)

        return_to_origin(hat, PORT_MOTOR)

    print(json.dumps(results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
