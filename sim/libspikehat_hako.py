"""
libspikehat_hako.py — Hakoniwa PDU バックエンドの SpikeHat 実装

libspikehat（実機）/ libspikehat_sim（MuJoCo スタンドアロン）と同じ API を提供し、
内部では hakopy PDU でセンサー・アクチュエーターを操作する。

sleep() は hakopy.usleep() を呼びシミュレーション時刻ベースで待機する。
これにより、コントローラーのロジックがウォール時刻に依存しなくなる。

将来の C ライブラリ版（libspikehat_hako.so）への移行を念頭に置き、
インターフェースを spikehat.h と揃えている。
"""

import sys
import math
import time
import hakopy

from pdu.python.std_msgs.pdu_conv_Float64 import pdu_to_py_Float64, py_to_pdu_Float64
from pdu.python.std_msgs.pdu_pytype_Float64 import Float64
from pdu.python.std_msgs.pdu_conv_Bool import pdu_to_py_Bool
from pdu.python.std_msgs.pdu_conv_ColorRGBA import pdu_to_py_ColorRGBA
from pdu.python.sensor_msgs.pdu_conv_Range import pdu_to_py_Range

# PDU チャンネル定数（SonarRadarAsset）
CH_RANGE         = 0
CH_COLOR_RGBA    = 1
CH_TURRET_TORQUE = 2
CH_MOTOR_ANGLE   = 3
CH_FORCE_SENSOR  = 4
PDU_SIZE_RANGE   = 184
PDU_SIZE_COLOR   = 40
PDU_SIZE_TORQUE  = 32
PDU_SIZE_ANGLE   = 32
PDU_SIZE_FORCE   = 28

DIST_INVALID_MM  = 2000

# デバイス種別定数（spikehat.h の enum に対応）
DEVICE_NONE     = 0
DEVICE_MOTOR_M  = 1
DEVICE_MOTOR_L  = 2
DEVICE_COLOR    = 3
DEVICE_DISTANCE = 4
DEVICE_FORCE    = 5


class HakoControllerStopped(Exception):
    """hakopy.usleep() が False を返したとき（シミュレーション停止）に送出する。"""
    pass


class HakoSpikeHat:
    """
    Hakoniwa 版 SpikeHat。
    sleep() が hakopy.usleep() を呼ぶことで、
    コントローラーのタイミングがシミュレーション時刻ベースになる。
    """

    # auto-press の押下維持時間（シミュレーション時刻、マイクロ秒）
    _AUTO_PRESS_DURATION_USEC = 150_000   # 150ms

    def __init__(self, robot_name: str):
        self._robot_name = robot_name
        self._sim_time_usec = 0
        # [0]=スタートボタン, [1]=ストップボタン
        self._auto_press_schedule_usec = [None, None]
        self._auto_press_end_usec      = [None, None]

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.motor_stop(0)

    def port_config(self, port, device_type):
        return 0

    # ── タイミング ──────────────────────────────────────────────────────────

    def sleep(self, seconds: float):
        """シミュレーション時刻で seconds 秒待機し、さらに同時間だけリアル時刻でも待機する。
        二重 sleep によりシミュレーション時刻を壁時計に追従させる（リアル時刻同期）。"""
        usec = int(max(0.0, seconds) * 1_000_000)
        if usec <= 0:
            return
        ok = hakopy.usleep(usec)
        if not ok:
            raise HakoControllerStopped("hakopy.usleep returned false")
        self._sim_time_usec += usec
        time.sleep(seconds)

    def schedule_auto_press(self, start_sec: float, stop_sec=None):
        """シミュレーション開始から start_sec 秒後にスタートボタン、
        stop_sec 秒後にストップボタンを自動注入するスケジュールを設定する。"""
        if start_sec is not None:
            self._auto_press_schedule_usec[0] = int(start_sec * 1_000_000)
            print(f"[libspikehat_hako] auto-start: {start_sec}s 後にスタートボタン注入",
                  file=sys.stderr)
        if stop_sec is not None:
            self._auto_press_schedule_usec[1] = int(stop_sec * 1_000_000)
            print(f"[libspikehat_hako] auto-stop:  {stop_sec}s 後にストップボタン注入",
                  file=sys.stderr)

    # ── モーター ────────────────────────────────────────────────────────────

    def motor_pwm(self, port, power: float):
        self._write_torque(max(-1.0, min(1.0, float(power))))

    def motor_stop(self, port):
        self._write_torque(0.0)

    def motor_start(self, port, speed: int):
        self._write_torque(max(-1.0, min(1.0, speed / 100.0)))

    def motor_run_to_position(self, port, position_deg: int, speed: int):
        """目標角度に到達するまで PWM で駆動する（シミュレーション時刻ベースのタイムアウト）。"""
        pwm          = (abs(speed) / 100.0) * 0.3
        timeout_usec = 5_000_000   # 5s
        start_usec   = self._sim_time_usec
        while self._sim_time_usec - start_usec < timeout_usec:
            current = self.motor_get_position(port)
            err = position_deg - current
            if abs(err) < 3.0:
                break
            self._write_torque(pwm if err > 0 else -pwm)
            self.sleep(0.05)
        self.motor_stop(port)

    def motor_get_position(self, port) -> int:
        raw = hakopy.pdu_read(self._robot_name, CH_MOTOR_ANGLE, PDU_SIZE_ANGLE)
        if raw:
            try:
                return int(pdu_to_py_Float64(bytearray(raw)).data)
            except Exception:
                pass
        return 0

    # ── センサー ────────────────────────────────────────────────────────────

    def distance_read(self, port) -> int:
        raw = hakopy.pdu_read(self._robot_name, CH_RANGE, PDU_SIZE_RANGE)
        if raw:
            try:
                r = pdu_to_py_Range(bytearray(raw))
                if r.range == float("inf") or r.range != r.range:
                    return DIST_INVALID_MM
                return int(r.range * 1000.0)
            except Exception:
                pass
        return DIST_INVALID_MM

    def color_read_hsv(self, port) -> tuple:
        raw = hakopy.pdu_read(self._robot_name, CH_COLOR_RGBA, PDU_SIZE_COLOR)
        if raw:
            try:
                c = pdu_to_py_ColorRGBA(bytearray(raw))
                return _rgb_to_hsv(c.r, c.g, c.b)
            except Exception:
                pass
        return (0, 0, 0)

    def color_read_rgb(self, port) -> tuple:
        raw = hakopy.pdu_read(self._robot_name, CH_COLOR_RGBA, PDU_SIZE_COLOR)
        if raw:
            try:
                c = pdu_to_py_ColorRGBA(bytearray(raw))
                return (int(c.r * 255), int(c.g * 255), int(c.b * 255))
            except Exception:
                pass
        return (0, 0, 0)

    def force_is_pressed(self, port) -> bool:
        now_usec = self._sim_time_usec
        # auto-press スケジュール処理
        for idx in (0, 1):
            if (self._auto_press_schedule_usec[idx] is not None
                    and now_usec >= self._auto_press_schedule_usec[idx]
                    and self._auto_press_end_usec[idx] is None):
                self._auto_press_schedule_usec[idx] = None
                self._auto_press_end_usec[idx] = now_usec + self._AUTO_PRESS_DURATION_USEC
                label = "スタート" if idx == 0 else "ストップ"
                print(f"[libspikehat_hako] auto-press: {label}ボタン注入 "
                      f"({self._AUTO_PRESS_DURATION_USEC // 1000}ms sim時刻)",
                      file=sys.stderr)
        # アクティブな押下ウィンドウ内なら True
        for idx in (0, 1):
            if self._auto_press_end_usec[idx] is not None:
                if now_usec < self._auto_press_end_usec[idx]:
                    return True
                else:
                    self._auto_press_end_usec[idx] = None
        # PDU から実際の force_sensor 状態を読む
        raw = hakopy.pdu_read(self._robot_name, CH_FORCE_SENSOR, PDU_SIZE_FORCE)
        if raw:
            try:
                return bool(pdu_to_py_Bool(bytearray(raw)).data)
            except Exception:
                pass
        return False

    # ── 内部ヘルパー ────────────────────────────────────────────────────────

    def _write_torque(self, pwm: float):
        try:
            f = Float64(); f.data = pwm
            raw = py_to_pdu_Float64(f)
            hakopy.pdu_write(self._robot_name, CH_TURRET_TORQUE, raw, len(raw))
        except Exception as e:
            print(f"[WARN] turret_torque write failed: {e}", file=sys.stderr)


def _rgb_to_hsv(r: float, g: float, b: float) -> tuple:
    """ColorRGBA の r,g,b (0〜1) を sonar_radar.py の HSV スケールに変換する。
    hue: 0〜360, sat: 0〜1000, val: 0〜1000"""
    mx = max(r, g, b)
    mn = min(r, g, b)
    delta = mx - mn
    val = int(round(mx * 1000.0))
    sat = int(round((delta / mx) * 1000.0)) if mx > 0 else 0
    if delta < 1e-6:
        return (0, sat, val)
    if mx == r:
        h = 60.0 * math.fmod((g - b) / delta, 6.0)
    elif mx == g:
        h = 60.0 * ((b - r) / delta + 2.0)
    else:
        h = 60.0 * ((r - g) / delta + 4.0)
    if h < 0:
        h += 360.0
    return (int(round(h)), sat, val)
