#!/usr/bin/env python3
"""
sonar_radar_ctrl_hako.py — Hakoniwa controller として sonar_radar.py を実行する

libspikehat_hako.HakoSpikeHat を hat として差し込み、
Hakoniwa の on_manual_timing_control コールバックで sonar_radar.py を直列実行する。

on_manual_timing_control を使うことで、コントローラーのタイミングが
シミュレーション時刻ベース（hakopy.usleep）になり、
コンダクターの実行速度に依存しない正確な時刻同期が実現できる。

PDU チャンネル（SonarRadarAsset）:
  CH 0: range         sensor_msgs/Range   (距離センサー ← 読む)
  CH 1: color_rgba    std_msgs/ColorRGBA  (カラーセンサー ← 読む)
  CH 2: turret_torque std_msgs/Float64    (旋回トルク指令 → 書く)
  CH 3: motor_angle   std_msgs/Float64    (モーター角度 ← 読む)
  CH 4: force_sensor  std_msgs/Bool       (フォースセンサー ← 読む)

使用方法:
  # ターミナル 1（plant 先に起動）
  bash run-hakopy.bash ~/Projects/sonar_radar/sim/sonar_radar_hako.py --viewer

  # ターミナル 2（controller）
  bash run-hakopy.bash ~/Projects/sonar_radar/sim/sonar_radar_ctrl_hako.py \\
    --auto-start 3 --auto-stop 20

  # ターミナル 3
  hako-cmd start
"""

import sys
import os
import argparse
import signal
import importlib.util
import types
import subprocess

# ─── パス設定 ────────────────────────────────────────────────────────────────

_here        = os.path.dirname(os.path.abspath(__file__))
_radar_file  = os.path.join(_here, "..", "raspi", "sonar_radar.py")
_pdu_py_dir  = os.path.join(_here, "libspikehat_sim", "python")

# libspikehat_hako.py（_here）と pdu Python バインディングをパスに追加
for _d in (_here, _pdu_py_dir):
    if _d not in sys.path:
        sys.path.insert(0, _d)

_PDU_FILENAME = "sonar-radar-pdudef-compact.json"
PDU_DEF_PATH = os.environ.get("SONAR_RADAR_PDU_DEF", "")
if not PDU_DEF_PATH or not os.path.exists(PDU_DEF_PATH):
    # カレントディレクトリ（run-hakopy.bash の実行場所 = hakoniwa-mujoco-robots/）を探す
    PDU_DEF_PATH = os.path.join(os.getcwd(), "config", _PDU_FILENAME)

# ─── インポート ───────────────────────────────────────────────────────────────

import hakopy
from libspikehat_hako import (
    HakoSpikeHat, HakoControllerStopped,
    DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE, DEVICE_FORCE,
)

# ─── 定数 ────────────────────────────────────────────────────────────────────

ASSET_NAME = "SonarRadarController"
ROBOT_NAME = "SonarRadarAsset"
STEP_USEC  = 1000   # conductor の delta と一致させる（plant も 1ms）

# ─── sonar_radar.py の差し替え実行 ────────────────────────────────────────────

def _make_sim_time_module(hat: HakoSpikeHat):
    """sonar_radar.py の time.monotonic() をシミュレーション時刻に差し替えたモジュールを返す。
    wait_for_force_release の MIN_PRESS_S チェックおよびタイムスタンプ出力が
    壁時計に依存するため必要。exec_module 前に sys.modules['time'] へ差し込む。"""
    import time as _real_time
    sim_time_mod = types.ModuleType("time")
    for _attr in dir(_real_time):
        setattr(sim_time_mod, _attr, getattr(_real_time, _attr))
    sim_time_mod.monotonic = lambda: hat._sim_time_usec / 1_000_000
    return sim_time_mod


def _run_sonar_radar(hat: HakoSpikeHat):
    """sonar_radar.py に HakoSpikeHat を差し込んで実行する。"""
    fake_spikehat = types.ModuleType("spikehat")
    fake_spikehat.SpikeHat        = lambda **_kw: hat
    fake_spikehat.DEVICE_MOTOR_L  = DEVICE_MOTOR_L
    fake_spikehat.DEVICE_FORCE    = DEVICE_FORCE
    fake_spikehat.DEVICE_COLOR    = DEVICE_COLOR
    fake_spikehat.DEVICE_DISTANCE = DEVICE_DISTANCE

    # exec_module 前に差し替えることで sonar_radar.py の import time が
    # シミュレーション時刻版を掴む
    import time as _real_time
    _orig_time = sys.modules.get("time")
    fake_time = _make_sim_time_module(hat)
    sys.modules["spikehat"] = fake_spikehat
    sys.modules["time"]     = fake_time

    try:
        spec = importlib.util.spec_from_file_location("sonar_radar", _radar_file)
        mod  = importlib.util.module_from_spec(spec)
        sys.modules["sonar_radar"] = mod
        spec.loader.exec_module(mod)
        mod.main()
    finally:
        # 他モジュールへの影響を避けるため time を元に戻す
        if _orig_time is not None:
            sys.modules["time"] = _orig_time
        else:
            del sys.modules["time"]


# ─── Hakoniwa コールバック ────────────────────────────────────────────────────

def on_initialize(_ctx):
    print("[INFO] ctrl on_initialize: OK", file=sys.stderr)
    return 0

def on_reset(_ctx):
    return 0

def _make_on_manual_timing_control(hat: HakoSpikeHat):
    def on_manual_timing_control(_ctx):
        try:
            _run_sonar_radar(hat)
        except HakoControllerStopped:
            print("[INFO] ctrl: シミュレーション停止を検出", file=sys.stderr)
        except KeyboardInterrupt:
            print("[INFO] ctrl: KeyboardInterrupt", file=sys.stderr)
        except Exception as e:
            print(f"[ERROR] ctrl: sonar_radar failed: {e}", file=sys.stderr)
            import traceback; traceback.print_exc(file=sys.stderr)

        print("[INFO] ctrl: sonar_radar 終了。hako-cmd stop/reset を発行します...",
              file=sys.stderr)
        for cmd in (["hako-cmd", "stop"], ["hako-cmd", "reset"]):
            try:
                subprocess.run(cmd, check=False, timeout=5)
                print(f"[INFO] {' '.join(cmd)} 完了", file=sys.stderr)
            except Exception as e:
                print(f"[WARN] {' '.join(cmd)} failed: {e}", file=sys.stderr)
        return 0
    return on_manual_timing_control


# ─── メイン ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="sonar_radar Hakoniwa controller")
    parser.add_argument("--auto-start", type=float, default=3.0, metavar="SEC",
                        help="シミュレーション開始から SEC 秒後にスタートボタンを注入（デフォルト: 3.0）")
    parser.add_argument("--auto-stop",  type=float, default=None, metavar="SEC",
                        help="スタートボタン注入から SEC 秒後にストップボタンを注入")
    args = parser.parse_args()

    if not os.path.exists(PDU_DEF_PATH):
        print(f"[ERROR] PDU def not found: {PDU_DEF_PATH}", file=sys.stderr)
        sys.exit(1)
    if not os.path.exists(_radar_file):
        print(f"[ERROR] sonar_radar.py not found: {_radar_file}", file=sys.stderr)
        sys.exit(1)

    hat = HakoSpikeHat(robot_name=ROBOT_NAME)

    # auto-press スケジュールをシミュレーション時刻（秒）で登録
    auto_start_sec = args.auto_start
    auto_stop_sec  = None
    if args.auto_stop is not None:
        auto_stop_sec = (auto_start_sec or 0.0) + args.auto_stop
    hat.schedule_auto_press(auto_start_sec, auto_stop_sec)

    cb = {
        "on_initialize":            on_initialize,
        "on_simulation_step":       None,
        "on_manual_timing_control": _make_on_manual_timing_control(hat),
        "on_reset":                 on_reset,
    }

    signal.signal(signal.SIGINT,  lambda s, f: os._exit(0))
    signal.signal(signal.SIGTERM, lambda s, f: os._exit(0))

    ret = hakopy.asset_register(
        ASSET_NAME, PDU_DEF_PATH, cb,
        STEP_USEC, hakopy.HAKO_ASSET_MODEL_CONTROLLER)
    if not ret:
        print("[ERROR] hakopy.asset_register failed", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] '{ASSET_NAME}' controller 登録完了。hako-cmd start を待機中...",
          file=sys.stderr)

    hakopy.start()
    print("[INFO] ctrl exiting.", file=sys.stderr)


if __name__ == "__main__":
    main()
