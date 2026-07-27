#!/usr/bin/env python3
"""
sonar_radar.py - レーダースキャナー（ステートマシン版）

SonarRadarSM クラスが1ティックだけ処理してすぐリターンする「開いたループ」を実装する。
外側ループ（system_driver）が tick() を繰り返し呼び、hat.sleep() で時間を進める。

  実機:      time.sleep() で進める → main() が担う
  Hakoniwa: hakopy.usleep() で進める → sonar_radar_ctrl_hako.py が担う
  sim:       hat.sleep() で MuJoCo ステップを進める → sonar_radar_sim.py が担う

ステートマシン状態:
  INIT → CALIBRATING → WAIT_FOR_START → SCANNING → WAIT_FOR_STOP → TERMINATED

ハードウェア構成:
  ポートA(0): Lアンギュラーモーター  - ドーム旋回（ギア減速 1:3、回転方向反転）
  ポートB(1): フォースセンサー       - スタート/ストップトリガー
  ポートC(2): カラーセンサー         - 旋回端マーカー検出（赤=左端, 青=右端）
  ポートD(3): 距離センサー           - 障害物計測
"""

import sys
import json
import time
import enum

# --- 実機専用設定 ---
try:
    from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_FORCE, DEVICE_COLOR, DEVICE_DISTANCE
except ImportError:
    import os
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'libspikehat', 'python'))
    from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_FORCE, DEVICE_COLOR, DEVICE_DISTANCE

# --- ハードウェア設定 ---
PORT_MOTOR    = 0
PORT_FORCE    = 1
PORT_COLOR    = 2
PORT_DISTANCE = 3

# --- 距離設定 ---
DIST_MIN_MM    = 50
DIST_MAX_MM    = 300
DIST_INVALID   = 2000
DIST_OFFSET_MM = 25

# --- モーター設定 ---
SCAN_PWM          = 0.1
ALIGN_SPEED       = 10
SAMPLE_INTERVAL_S = 0.05

# --- ギア比 ---
GEAR_RATIO         = 3
SENSOR_HOME_OFFSET = 5

# フォースセンサー: 有効押下の最低ティック数（MIN_PRESS = 0.1s / SAMPLE_INTERVAL_S）
MIN_PRESS_TICKS = 2


def dome_to_motor(dome_deg):
    return -dome_deg * GEAR_RATIO


def motor_to_dome(motor_deg):
    return -motor_deg / GEAR_RATIO


# --- カラー判定 ---
RED_SAT_MIN  = 40
RED_VAL_MIN  = 40
BLUE_HUE_LO  = 210
BLUE_HUE_HI  = 270
BLUE_SAT_MIN = 580
BLUE_VAL_MIN = 100


def is_red(hue, sat, val):
    if sat < RED_SAT_MIN or val < RED_VAL_MIN:
        return False
    return hue >= 340 or hue <= 20


def is_blue(hue, sat, val):
    if sat < BLUE_SAT_MIN or val < BLUE_VAL_MIN:
        return False
    return BLUE_HUE_LO <= hue <= BLUE_HUE_HI


def filter_distance(mm):
    if mm == DIST_INVALID:
        return None
    corrected = mm + DIST_OFFSET_MM
    if corrected < DIST_MIN_MM or corrected > DIST_MAX_MM:
        return None
    return corrected


# ─── ステートマシン ────────────────────────────────────────────────────────────

class State(enum.Enum):
    INIT              = 'INIT'
    CALIB_TO_ZERO     = 'CALIB_TO_ZERO'     # 待つもの: モーターが機械的0位置に到達
    CALIB_TO_OFFSET   = 'CALIB_TO_OFFSET'   # 待つもの: モーターがSENSOR_HOME_OFFSET位置に到達
    WAIT_FOR_START    = 'WAIT_FOR_START'    # 待つもの: フォースセンサーの押下→解放
    SCANNING          = 'SCANNING'          # 待つもの: フォースセンサーの押下→解放
    RETURN_TO_ORIGIN  = 'RETURN_TO_ORIGIN'  # 待つもの: モーターがzero_posに到達
    TERMINATED        = 'TERMINATED'


class SonarRadarSM:
    """
    sonar_radar のステートマシン（フラット1段）。
    tick(hat) を呼ぶたびに1ステップ処理してすぐリターンする。
    外側ループが hat.sleep() で時間を進める責任を持つ。

    clock: 経過時間（秒）を返す callable。省略時は time.monotonic。
           Hakoniwa では lambda: hat._sim_time_usec / 1e6 を渡す。
    """

    def __init__(self, clock=None):
        self._clock       = clock if clock is not None else time.monotonic
        self.state        = State.INIT
        self.results      = []
        self._zero_pos    = 0
        self._force_on    = False  # クリック検出用（共通）
        self._press_ticks = 0      # クリック検出用（共通）
        self._scan_pwm    = SCAN_PWM
        self._on_marker   = False

    def tick(self, hat):
        if   self.state == State.INIT:             self._tick_init(hat)
        elif self.state == State.CALIB_TO_ZERO:    self._tick_calib_to_zero(hat)
        elif self.state == State.CALIB_TO_OFFSET:  self._tick_calib_to_offset(hat)
        elif self.state == State.WAIT_FOR_START:   self._tick_wait_for_start(hat)
        elif self.state == State.SCANNING:         self._tick_scanning(hat)
        elif self.state == State.RETURN_TO_ORIGIN: self._tick_return_to_origin(hat)

    def is_terminated(self):
        return self.state == State.TERMINATED

    # ── INIT ──────────────────────────────────────────────────────────────────

    def _tick_init(self, hat):
        hat.port_config(PORT_MOTOR,    DEVICE_MOTOR_L)
        hat.port_config(PORT_FORCE,    DEVICE_FORCE)
        hat.port_config(PORT_COLOR,    DEVICE_COLOR)
        hat.port_config(PORT_DISTANCE, DEVICE_DISTANCE)
        print(f"[{self._clock():6.2f}s] キャリブレーション: 機械的0位置へ移動...", file=sys.stderr)
        self.state = State.CALIB_TO_ZERO

    # ── CALIB_TO_ZERO ─────────────────────────────────────────────────────────
    # 待つもの: モーターが機械的0位置（encoder=0）に到達

    def _tick_calib_to_zero(self, hat):
        if self._drive_to(hat, 0, ALIGN_SPEED):
            offset = round(dome_to_motor(SENSOR_HOME_OFFSET))
            print(f"[{self._clock():6.2f}s] SENSOR_HOME_OFFSET(dome {SENSOR_HOME_OFFSET}度 = motor {offset}度)分を補正...",
                  file=sys.stderr)
            self.state = State.CALIB_TO_OFFSET

    # ── CALIB_TO_OFFSET ───────────────────────────────────────────────────────
    # 待つもの: モーターがSENSOR_HOME_OFFSET位置に到達 → zero_pos を記録

    def _tick_calib_to_offset(self, hat):
        offset = round(dome_to_motor(SENSOR_HOME_OFFSET))
        if self._drive_to(hat, offset, ALIGN_SPEED):
            self._zero_pos    = hat.motor_get_position(PORT_MOTOR)
            self._force_on    = False
            self._press_ticks = 0
            print(f"[{self._clock():6.2f}s] キャリブレーション完了 (現在位置 = 0°, encoder={self._zero_pos})",
                  file=sys.stderr)
            print(f"[{self._clock():6.2f}s] フォースセンサーを押して離すとスキャン開始します...", file=sys.stderr)
            self.state = State.WAIT_FOR_START

    # ── WAIT_FOR_START ────────────────────────────────────────────────────────
    # 待つもの: フォースセンサーのクリック（押下→解放）

    def _tick_wait_for_start(self, hat):
        if self._detect_force_click(hat):
            print(f"[{self._clock():6.2f}s] スキャン開始", file=sys.stderr)
            self._scan_pwm  = SCAN_PWM
            self._on_marker = False
            hat.motor_pwm(PORT_MOTOR, self._scan_pwm)
            print(f"連続スキャン開始: 速度(PWM)={self._scan_pwm}, "
                  f"間隔={SAMPLE_INTERVAL_S*1000:.0f}ms", file=sys.stderr)
            self.state = State.SCANNING

    # ── SCANNING ──────────────────────────────────────────────────────────────
    # 待つもの: フォースセンサーの押下→解放

    def _tick_scanning(self, hat):
        # マーカー検出
        try:
            h, s, v = hat.color_read_hsv(PORT_COLOR)
            marker = is_red(h, s, v) or is_blue(h, s, v)
            if marker and not self._on_marker:
                name = "赤" if is_red(h, s, v) else "青"
                print(f"{name}マーカー検出: 反転します", file=sys.stderr)
                self._scan_pwm = -self._scan_pwm
                hat.motor_pwm(PORT_MOTOR, self._scan_pwm)
            self._on_marker = marker
        except RuntimeError:
            pass

        # 角度・距離を記録
        try:
            angle      = hat.motor_get_position(PORT_MOTOR) - self._zero_pos
            dome_angle = motor_to_dome(angle)
        except RuntimeError:
            angle = dome_angle = None

        try:
            dist = filter_distance(hat.distance_read(PORT_DISTANCE))
        except RuntimeError:
            dist = None

        self.results.append({"angle": angle, "dome_angle": dome_angle, "distance_mm": dist})

        label     = f"{dist:5d} mm" if dist is not None else "  null"
        angle_str = f"{angle:+4d}" if angle is not None else "  --"
        dome_str  = f"{dome_angle:+6.1f}" if dome_angle is not None else "    --"
        print(f"[{self._clock():6.2f}s] motor:{angle_str}° dome:{dome_str}° -> {label}",
              file=sys.stderr)

        # フォースセンサーのクリックでスキャン終了
        if self._detect_force_click(hat):
            print(f"[{self._clock():6.2f}s] フォースセンサー: スキャン終了", file=sys.stderr)
            hat.motor_stop(PORT_MOTOR)
            print(f"[{self._clock():6.2f}s] 0位置へ復帰: zero_pos={self._zero_pos}", file=sys.stderr)
            self.state = State.RETURN_TO_ORIGIN

    # ── RETURN_TO_ORIGIN ──────────────────────────────────────────────────────
    # 待つもの: モーターがzero_posに到達

    def _tick_return_to_origin(self, hat):
        if self._drive_to(hat, self._zero_pos, ALIGN_SPEED):
            print(f"[{self._clock():6.2f}s] 0位置へ復帰完了", file=sys.stderr)
            self.state = State.TERMINATED

    # ── 内部ヘルパー ──────────────────────────────────────────────────────────

    def _detect_force_click(self, hat) -> bool:
        """フォースセンサーのクリック（押下→解放）を検出する。毎ティック呼ぶこと。
        MIN_PRESS_TICKS 以上の押下の後に解放されたとき True を返す。"""
        try:
            pressed = hat.force_is_pressed(PORT_FORCE)
        except RuntimeError:
            return False
        if pressed and not self._force_on:
            self._force_on    = True
            self._press_ticks = 1
        elif pressed and self._force_on:
            self._press_ticks += 1
        elif not pressed and self._force_on:
            self._force_on = False
            if self._press_ticks >= MIN_PRESS_TICKS:
                self._press_ticks = 0
                return True
            self._press_ticks = 0
        return False

    def _drive_to(self, hat, target: int, speed: int) -> bool:
        """目標角度に向けて1ティック分トルクを適用する。到達したら True を返す。"""
        try:
            current = hat.motor_get_position(PORT_MOTOR)
        except RuntimeError:
            return False
        err = target - current
        if abs(err) < 3:
            hat.motor_stop(PORT_MOTOR)
            return True
        pwm = (abs(speed) / 100.0) * 0.3
        hat.motor_pwm(PORT_MOTOR, pwm if err > 0 else -pwm)
        return False


# ─── 実機用エントリポイント ────────────────────────────────────────────────────

def main():
    t_process_start = time.monotonic()
    try:
        hat_instance = SpikeHat()
    except RuntimeError:
        print("エラー: Build HAT ファームウェアがロードされていません。", file=sys.stderr)
        sys.exit(1)
    print(f"[SpikeHat()構築: {time.monotonic() - t_process_start:.2f}s]", file=sys.stderr)

    start_time = time.monotonic()

    with hat_instance as hat:
        hat.sleep(1.0)
        sm = SonarRadarSM(clock=lambda: time.monotonic() - start_time)
        while not sm.is_terminated():
            sm.tick(hat)
            hat.sleep(SAMPLE_INTERVAL_S)

    print(json.dumps(sm.results, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
