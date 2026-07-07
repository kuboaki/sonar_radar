#!/usr/bin/env python3
"""
sonar_radar_ctrl_hako.py — Hakoniwa controller として sonar_radar.py を実行する

sonar_radar.py の SpikeHat を PDU バックエンドに差し替えて、
Hakoniwa の Controller Asset として動作させる。

on_simulation_step コールバックが最初に呼ばれた時点で sonar_radar.py を別スレッドで起動する。
on_simulation_step を使うことで PDU sync に参加でき、plant 側の SYNC MODE ブロックを回避できる。

PDU チャンネル（SonarRadarAsset）:
  CH 0: range         sensor_msgs/Range   (距離センサー ← 読む)
  CH 1: color_rgba    std_msgs/ColorRGBA  (カラーセンサー ← 読む)
  CH 2: turret_torque std_msgs/Float64    (旋回トルク指令 → 書く)
  CH 3: motor_angle   std_msgs/Float64    (モーター角度 ← 読む)
  CH 4: force_sensor  std_msgs/Bool       (フォースセンサー ← 読む)

使用方法:
  # ターミナル 1（plant 先に起動）
  SONAR_RADAR_PDU_DEF=config/sonar-radar-pdudef-compact.json \\
    bash run-hakopy.bash ~/Projects/sonar_radar/sim/sonar_radar_hako.py --viewer --debug

  # ターミナル 2（controller）
  SONAR_RADAR_PDU_DEF=config/sonar-radar-pdudef-compact.json \\
    bash run-hakopy.bash ~/Projects/sonar_radar/sim/sonar_radar_ctrl_hako.py \\
    --auto-start 3 --auto-stop 20

  # ターミナル 3
  hako-cmd start
"""

import sys
import os
import argparse
import time as _time
import math
import json
import signal
import threading
import importlib.util
import types

# ─── パス設定 ────────────────────────────────────────────────────────────────

_here       = os.path.dirname(os.path.abspath(__file__))
_radar_file = os.path.join(_here, "..", "raspi", "sonar_radar.py")

_hako_robots_dir = os.path.join(_here, "..", "..", "..", "..")
PDU_DEF_PATH = os.path.join(_hako_robots_dir, "config", "sonar-radar-pdudef-compact.json")
if not os.path.exists(PDU_DEF_PATH):
    PDU_DEF_PATH = os.environ.get("SONAR_RADAR_PDU_DEF", PDU_DEF_PATH)

# ─── インポート ───────────────────────────────────────────────────────────────

import hakopy
from pdu.python.std_msgs.pdu_conv_Float64 import pdu_to_py_Float64, py_to_pdu_Float64
from pdu.python.std_msgs.pdu_pytype_Float64 import Float64
from pdu.python.std_msgs.pdu_conv_Bool import pdu_to_py_Bool
from pdu.python.std_msgs.pdu_conv_ColorRGBA import pdu_to_py_ColorRGBA
from pdu.python.sensor_msgs.pdu_conv_Range import pdu_to_py_Range

# ─── 定数 ────────────────────────────────────────────────────────────────────

ASSET_NAME = "SonarRadarController"   # plant の "SonarRadarAsset" とは別名
ROBOT_NAME = "SonarRadarAsset"        # PDU アクセス時のロボット名

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
STEP_USEC        = 1000   # conductor の delta と一致させる（plant も 1ms）

# ─── 自動注入スケジュール ──────────────────────────────────────────────────────

_AUTO_PRESS_DURATION = 0.15   # 押下を維持する秒数（sonar_radar.py の MIN_PRESS_S=0.1 を超える値）
_auto_press_schedule = [None, None]
_auto_press_end      = [None, None]

# ─── PDU バックエンド SpikeHat ────────────────────────────────────────────────

class PduSpikeHat:
    """
    sonar_radar.py の SpikeHat API を PDU 経由で実装するアダプター。
    実機用 SpikeHat / libspikehat_sim と同じインターフェースを提供する。
    """

    def __init__(self):
        pass

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.motor_stop(0)

    def port_config(self, port, device_type):
        return 0

    def sleep(self, seconds: float):
        _time.sleep(seconds)

    # --- モーター ---

    def motor_pwm(self, port, power: float):
        self._write_torque(max(-1.0, min(1.0, float(power))))

    def motor_stop(self, port):
        self._write_torque(0.0)

    def motor_start(self, port, speed: int):
        self._write_torque(max(-1.0, min(1.0, speed / 100.0)))

    def motor_run_to_position(self, port, position_deg: int, speed: int):
        """目標角度に到達するまで PWM で駆動する（PDU ベースの簡易実装）"""
        pwm     = (abs(speed) / 100.0) * 0.3
        timeout = 5.0
        start   = _time.monotonic()
        while _time.monotonic() - start < timeout:
            current = self.motor_get_position(port)
            err = position_deg - current
            if abs(err) < 3.0:
                break
            self._write_torque(pwm if err > 0 else -pwm)
            _time.sleep(0.05)
        self.motor_stop(port)

    def motor_get_position(self, port) -> int:
        raw = hakopy.pdu_read(ROBOT_NAME, CH_MOTOR_ANGLE, PDU_SIZE_ANGLE)
        if raw:
            try:
                return int(pdu_to_py_Float64(bytearray(raw)).data)
            except Exception:
                pass
        return 0

    # --- センサー ---

    def distance_read(self, port) -> int:
        raw = hakopy.pdu_read(ROBOT_NAME, CH_RANGE, PDU_SIZE_RANGE)
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
        raw = hakopy.pdu_read(ROBOT_NAME, CH_COLOR_RGBA, PDU_SIZE_COLOR)
        if raw:
            try:
                c = pdu_to_py_ColorRGBA(bytearray(raw))
                return _rgb_to_hsv(c.r, c.g, c.b)
            except Exception:
                pass
        return (0, 0, 0)

    def color_read_rgb(self, port) -> tuple:
        raw = hakopy.pdu_read(ROBOT_NAME, CH_COLOR_RGBA, PDU_SIZE_COLOR)
        if raw:
            try:
                c = pdu_to_py_ColorRGBA(bytearray(raw))
                return (int(c.r * 255), int(c.g * 255), int(c.b * 255))
            except Exception:
                pass
        return (0, 0, 0)

    def force_is_pressed(self, port) -> bool:
        now = _time.monotonic()
        # 自動注入スケジュールを処理
        for idx in (0, 1):
            if (_auto_press_schedule[idx] is not None
                    and now >= _auto_press_schedule[idx]
                    and _auto_press_end[idx] is None):
                _auto_press_schedule[idx] = None
                _auto_press_end[idx] = now + _AUTO_PRESS_DURATION
                label = "スタート" if idx == 0 else "ストップ"
                print(f"[ctrl] auto-press: {label}ボタン注入 ({_AUTO_PRESS_DURATION*1000:.0f}ms)",
                      file=sys.stderr)
        # アクティブな押下ウィンドウ内なら True
        for idx in (0, 1):
            if _auto_press_end[idx] is not None:
                if now < _auto_press_end[idx]:
                    return True
                else:
                    _auto_press_end[idx] = None
        # PDU から実際の force_sensor 状態を読む
        raw = hakopy.pdu_read(ROBOT_NAME, CH_FORCE_SENSOR, PDU_SIZE_FORCE)
        if raw:
            try:
                return bool(pdu_to_py_Bool(bytearray(raw)).data)
            except Exception:
                pass
        return False

    def _write_torque(self, pwm: float):
        try:
            f = Float64(); f.data = pwm
            raw = py_to_pdu_Float64(f)
            hakopy.pdu_write(ROBOT_NAME, CH_TURRET_TORQUE, raw, len(raw))
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


# ─── sonar スレッド本体 ──────────────────────────────────────────────────────

def _sonar_thread_main():
    """sonar_radar.py を実行し、完了後に hako-cmd stop/reset を発行する。"""
    import subprocess
    print("[INFO] sonar スレッド開始", file=sys.stderr)
    try:
        _run_sonar_radar()
    except Exception as e:
        print(f"[ERROR] sonar_radar failed: {e}", file=sys.stderr)
        import traceback; traceback.print_exc(file=sys.stderr)
    print("[INFO] sonar スレッド終了。シミュレーション停止を要求します...", file=sys.stderr)
    for cmd in (["hako-cmd", "stop"], ["hako-cmd", "reset"]):
        try:
            subprocess.run(cmd, check=False, timeout=5)
            print(f"[INFO] {' '.join(cmd)} 完了", file=sys.stderr)
            _time.sleep(1.0)
        except Exception as e:
            print(f"[WARN] {' '.join(cmd)} failed: {e}", file=sys.stderr)


# ─── sonar_radar.py の差し替え実行 ────────────────────────────────────────────

def _run_sonar_radar():
    """sonar_radar.py を PduSpikeHat を差し込んで実行し、結果を stdout に出力する。"""
    fake_mod = types.ModuleType("spikehat")
    fake_mod.SpikeHat       = PduSpikeHat
    fake_mod.DEVICE_MOTOR_L  = 2
    fake_mod.DEVICE_FORCE    = 5
    fake_mod.DEVICE_COLOR    = 3
    fake_mod.DEVICE_DISTANCE = 4
    sys.modules["spikehat"] = fake_mod

    spec = importlib.util.spec_from_file_location("sonar_radar", _radar_file)
    mod  = importlib.util.module_from_spec(spec)
    sys.modules["sonar_radar"] = mod
    spec.loader.exec_module(mod)
    mod.main()


# ─── Hakoniwa コールバック ────────────────────────────────────────────────────

# on_simulation_step で sonar_radar.py を別スレッドで起動する。
# on_manual_timing_control を使わず on_simulation_step を使うことで、
# PDU sync（notify_write_pdu_done）に参加でき、plant 側の SYNC MODE ブロックが解消される。

_sonar_thread  = None
_sonar_started = False


def on_initialize(_ctx):
    print("[INFO] ctrl on_initialize: OK", file=sys.stderr)
    return 0

def on_reset(_ctx):
    global _sonar_thread, _sonar_started
    _sonar_started = False
    _sonar_thread  = None
    return 0

def on_simulation_step(_ctx):
    global _sonar_thread, _sonar_started
    if not _sonar_started:
        _sonar_started = True
        _sonar_thread = threading.Thread(target=_sonar_thread_main, daemon=True, name="sonar")
        _sonar_thread.start()
    return 0

cb = {
    "on_initialize":            on_initialize,
    "on_simulation_step":       on_simulation_step,
    "on_manual_timing_control": None,
    "on_reset":                 on_reset,
}

# ─── メイン ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="sonar_radar Hakoniwa controller")
    parser.add_argument("--auto-start", type=float, default=3.0, metavar="SEC",
                        help="hako-cmd start から SEC 秒後にスキャン開始ボタンを注入（デフォルト: 3.0）")
    parser.add_argument("--auto-stop",  type=float, default=None, metavar="SEC",
                        help="スキャン開始から SEC 秒後に停止ボタンを注入")
    args = parser.parse_args()

    if not os.path.exists(PDU_DEF_PATH):
        print(f"[ERROR] PDU def not found: {PDU_DEF_PATH}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(_radar_file):
        print(f"[ERROR] sonar_radar.py not found: {_radar_file}", file=sys.stderr)
        sys.exit(1)

    _auto_start_sec = args.auto_start
    _auto_stop_sec  = args.auto_stop

    # on_simulation_step が最初に呼ばれた時点でスケジュールを設定するラッパー
    _orig_on_step = on_simulation_step

    def on_simulation_step_with_schedule(_ctx):
        global _sonar_started
        if not _sonar_started:
            now = _time.monotonic()
            if _auto_start_sec is not None:
                _auto_press_schedule[0] = now + _auto_start_sec
                print(f"[INFO] auto-start: {_auto_start_sec}秒後にスキャン開始ボタンを注入", file=sys.stderr)
            if _auto_stop_sec is not None:
                offset = (_auto_start_sec or 0) + _auto_stop_sec
                _auto_press_schedule[1] = now + offset
                print(f"[INFO] auto-stop:  {offset}秒後に停止ボタンを注入", file=sys.stderr)
        return _orig_on_step(_ctx)

    cb["on_simulation_step"] = on_simulation_step_with_schedule

    def _shutdown(reason: str):
        print(f"[INFO] ctrl shutdown: {reason}", file=sys.stderr, flush=True)
        os._exit(0)

    signal.signal(signal.SIGINT,  lambda s, f: _shutdown(f"signal {s}"))
    signal.signal(signal.SIGTERM, lambda s, f: _shutdown(f"signal {s}"))

    # controller は conductor_start を呼ばない（plant が担当）
    ret = hakopy.asset_register(
        ASSET_NAME, PDU_DEF_PATH, cb,
        STEP_USEC, hakopy.HAKO_ASSET_MODEL_CONTROLLER)
    if not ret:
        print("[ERROR] hakopy.asset_register failed", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] '{ASSET_NAME}' controller 登録完了。hako-cmd start を待機中...",
          file=sys.stderr)

    hakopy.start()
    # _sonar_thread_main が hako-cmd stop/reset を発行済みなのでここでは何もしない
    print("[INFO] ctrl exiting.", file=sys.stderr)


if __name__ == "__main__":
    main()
