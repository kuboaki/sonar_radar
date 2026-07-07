#!/usr/bin/env python3
"""
sonar_radar_ctrl_hako.py — Hakoniwa controller として SonarRadarSM を実行する

SonarRadarSM（sonar_radar.py のステートマシン）に HakoSpikeHat を差し込み、
on_manual_timing_control コールバック内で「開いたループ」を駆動する。

on_manual_timing_control の中で:
  while not sm.is_terminated():
      sm.tick(hat)        # 1ティック処理
      hat.sleep(INTERVAL) # hakopy.usleep() でシミュレーション時刻を進める

これにより、コントローラーのタイミングがシミュレーション時刻ベースになり、
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
import types
import subprocess

# ─── パス設定 ────────────────────────────────────────────────────────────────

_here        = os.path.dirname(os.path.abspath(__file__))
_raspi_dir   = os.path.join(_here, "..", "raspi")
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

# sonar_radar.py を import する前に spikehat スタブを差し込む
# （sonar_radar.py のモジュールレベルで "from spikehat import ..." が実行されるため）
_fake_spikehat = types.ModuleType("spikehat")
_fake_spikehat.SpikeHat        = object   # main() は呼ばないので stub で十分
_fake_spikehat.DEVICE_MOTOR_L  = DEVICE_MOTOR_L
_fake_spikehat.DEVICE_FORCE    = DEVICE_FORCE
_fake_spikehat.DEVICE_COLOR    = DEVICE_COLOR
_fake_spikehat.DEVICE_DISTANCE = DEVICE_DISTANCE
sys.modules.setdefault("spikehat", _fake_spikehat)

if _raspi_dir not in sys.path:
    sys.path.insert(0, _raspi_dir)

from sonar_radar import SonarRadarSM, SAMPLE_INTERVAL_S

# ─── 定数 ────────────────────────────────────────────────────────────────────

ASSET_NAME = "SonarRadarController"
ROBOT_NAME = "SonarRadarAsset"
STEP_USEC  = 1000   # conductor の delta と一致させる（plant も 1ms）

# ─── SonarRadarSM を HakoSpikeHat で駆動する ────────────────────────────────

def _run_sonar_radar(hat: HakoSpikeHat):
    """SonarRadarSM を HakoSpikeHat で駆動する。終了したら results を返す。"""
    sm = SonarRadarSM(clock=lambda: hat._sim_time_usec / 1_000_000)
    while not sm.is_terminated():
        sm.tick(hat)
        hat.sleep(SAMPLE_INTERVAL_S)
    return sm.results


# ─── Hakoniwa コールバック ────────────────────────────────────────────────────

def on_initialize(_ctx):
    print("[INFO] ctrl on_initialize: OK", file=sys.stderr)
    return 0

def on_reset(_ctx):
    return 0

def _make_on_manual_timing_control(hat: HakoSpikeHat):
    def on_manual_timing_control(_ctx):
        try:
            results = _run_sonar_radar(hat)
            print(f"[INFO] ctrl: スキャン完了 ({len(results)} サンプル)", file=sys.stderr)
        except HakoControllerStopped:
            print("[INFO] ctrl: シミュレーション停止を検出", file=sys.stderr)
        except KeyboardInterrupt:
            print("[INFO] ctrl: KeyboardInterrupt", file=sys.stderr)
        except Exception as e:
            print(f"[ERROR] ctrl: SonarRadarSM failed: {e}", file=sys.stderr)
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
