#!/usr/bin/env python3
"""
test_stop_at_marker.py - マーカーで停止するテストスクリプト

0位置から旋回し、マーカー（負方向=青、正方向=赤）を検出した時点でモーターを停止する。
オーバーシュート込みで実際のdome位置とモーター角度の対応を観測するための簡易ツール。

使い方:
  python3 test_stop_at_marker.py            # 負方向に旋回、青マーカーで停止（既定）
  python3 test_stop_at_marker.py --positive # 正方向に旋回、赤マーカーで停止
"""

import sys
import argparse
sys.path.insert(0, '/home/kuboaki/projects/libspikehat/python')

from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_COLOR

PORT_MOTOR = 0   # ポートA
PORT_COLOR = 2   # ポートC

SCAN_PWM    = 0.1
ALIGN_SPEED = 10

# ギア比（モーター:dome = 1:3、回転方向反転）
GEAR_RATIO = 3

# --- カラー判定 ---
RED_SAT_MIN  = 40
RED_VAL_MIN  = 40
BLUE_HUE_LO  = 210
BLUE_HUE_HI  = 270
BLUE_SAT_MIN = 580
BLUE_VAL_MIN = 100


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


def main():
    parser = argparse.ArgumentParser(description="青マーカーで停止するテスト")
    parser.add_argument("--positive", action="store_true",
                         help="正方向に旋回する（既定は負方向）")
    args = parser.parse_args()

    scan_pwm = SCAN_PWM if args.positive else -SCAN_PWM

    try:
        hat_instance = SpikeHat()
    except RuntimeError:
        print("エラー: Build HAT ファームウェアがロードされていません。", file=sys.stderr)
        print("先に以下を実行してください:", file=sys.stderr)
        print("  python3 -c \"from buildhat import Motor; Motor('A')\"", file=sys.stderr)
        sys.exit(1)

    with hat_instance as hat:
        hat.port_config(PORT_MOTOR, DEVICE_MOTOR_L)
        hat.port_config(PORT_COLOR, DEVICE_COLOR)
        hat.sleep(1.0)

        return_to_origin(hat, PORT_MOTOR)

        print(f"旋回開始: PWM={scan_pwm}", file=sys.stderr)
        hat.motor_pwm(PORT_MOTOR, scan_pwm)

        marker_name = "赤" if args.positive else "青"
        marker_check = is_red if args.positive else is_blue

        while True:
            try:
                h, s, v = hat.color_read_hsv(PORT_COLOR)
                if marker_check(h, s, v):
                    print(f"{marker_name}マーカー検出: 停止します", file=sys.stderr)
                    break
            except RuntimeError:
                pass
            hat.sleep(0.01)

        hat.motor_stop(PORT_MOTOR)
        hat.sleep(0.3)

        try:
            motor_angle = hat.motor_get_position(PORT_MOTOR)
        except RuntimeError:
            motor_angle = None

        if motor_angle is not None:
            dome_angle = -motor_angle / GEAR_RATIO
            print(f"停止時モーター角度: {motor_angle:+d}°", file=sys.stderr)
            print(f"計算上のdome角度  : {dome_angle:+.1f}°", file=sys.stderr)
            print("実物のdome位置と上記の計算値を比較してください。", file=sys.stderr)
        else:
            print("モーター角度の取得に失敗しました。", file=sys.stderr)


if __name__ == "__main__":
    main()
