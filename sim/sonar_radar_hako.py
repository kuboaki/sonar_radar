#!/usr/bin/env python3
"""
sonar_radar_hako.py — Hakoniwa plant として sonar_radar を MuJoCo シミュレーションで動かす

sonar_radar_sim.py の「閉じたループ」を「開いたループ」に変換し、
Hakoniwa の on_simulation_step コールバックで1ステップずつ進める。
センサー測定は libspikehat_sim に委ね、結果を Hakoniwa PDU に書く。

PDU チャンネル（SonarRadarAsset）:
  CH 0: range         sensor_msgs/Range   (距離センサー → 書く)
  CH 1: color_rgba    std_msgs/ColorRGBA  (カラーセンサー → 書く)
  CH 2: turret_torque std_msgs/Float64    (旋回トルク指令 → 読む)
  CH 3: motor_angle   std_msgs/Float64    (モーター角度  → 書く)
  CH 4: force_sensor  std_msgs/Bool       (フォースセンサー → 書く)

構造:
  --viewer なし : メインスレッドで hakopy.start() を直接呼ぶ
  --viewer あり : sonar_radar_viewer.py を mjpython の別プロセスで起動し、
                  on_simulation_step ごとに /tmp/sonar_radar_qpos.bin へ qpos を書き込む。
                  viewer プロセスはファイルを監視し、更新が止まれば自動終了する。
                  （GIL の問題を完全に回避できる）

使用方法:
  cd ~/Projects/hakoniwa-mujoco-robots
  # viewer なし
  bash run-hakopy.bash ~/Projects/sonar_radar/sim/sonar_radar_hako.py --debug
  # viewer あり（run-hakopy.bash が mjpython で起動する）
  bash run-hakopy.bash ~/Projects/sonar_radar/sim/sonar_radar_hako.py --viewer --debug

環境変数:
  SPIKEHAT_SIM_XML     : MuJoCo XML ファイルのパス
  SONAR_RADAR_PDU_DEF  : PDU 定義 JSON のパス
"""

import sys
import os
import argparse
import math
import struct
import subprocess as _subprocess
import time as _time
import signal

# ─── パス設定 ────────────────────────────────────────────────────────────────

_here        = os.path.dirname(os.path.abspath(__file__))
_lib_sim_dir = os.path.join(_here, "libspikehat_sim", "python")
_xml_default = os.path.join(_here, "..", "mujoco_model", "sonar_radar.xml")
_xml_path    = os.environ.get("SPIKEHAT_SIM_XML", os.path.realpath(_xml_default))

sys.path.insert(0, _lib_sim_dir)

# ─── インポート ───────────────────────────────────────────────────────────────

import hakopy
from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE, DEVICE_FORCE
from pdu.python.std_msgs.pdu_conv_ColorRGBA import py_to_pdu_ColorRGBA
from pdu.python.std_msgs.pdu_pytype_ColorRGBA import ColorRGBA
from pdu.python.std_msgs.pdu_conv_Float64 import pdu_to_py_Float64, py_to_pdu_Float64
from pdu.python.std_msgs.pdu_pytype_Float64 import Float64
from pdu.python.std_msgs.pdu_conv_Bool import py_to_pdu_Bool
from pdu.python.std_msgs.pdu_pytype_Bool import Bool
from pdu.python.sensor_msgs.pdu_conv_Range import py_to_pdu_Range
from pdu.python.sensor_msgs.pdu_pytype_Range import Range

# ─── 定数 ────────────────────────────────────────────────────────────────────

ASSET_NAME   = "SonarRadarAsset"
ROBOT_NAME   = "SonarRadarAsset"

_PDU_FILENAME = "sonar-radar-pdudef-compact.json"
PDU_DEF_PATH = os.environ.get("SONAR_RADAR_PDU_DEF", "")
if not PDU_DEF_PATH or not os.path.exists(PDU_DEF_PATH):
    # カレントディレクトリ（run-hakopy.bash の実行場所 = hakoniwa-mujoco-robots/）を探す
    _cwd_candidate = os.path.join(os.getcwd(), "config", _PDU_FILENAME)
    PDU_DEF_PATH = _cwd_candidate

PORT_MOTOR    = 0
PORT_FORCE    = 1
PORT_COLOR    = 2
PORT_DISTANCE = 3

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
DIST_MIN_M       = 0.05
DIST_MAX_M       = 0.30

# ─── 状態 ────────────────────────────────────────────────────────────────────

_state = {
    "hat":        None,
    "debug":      False,
    "step_count": 0,
    "nq":         0,
}

_DEBUG_INTERVAL = 100
_QPOS_FILE      = "/tmp/sonar_radar_qpos.bin"


def _write_qpos(hat, nq: int):
    try:
        vals = [hat.sim_get_qpos(i) for i in range(nq)]
        data = struct.pack(f"{nq}d", *vals)
        with open(_QPOS_FILE, "wb") as f:
            f.write(data)
    except Exception:
        pass

# ─── コールバック ─────────────────────────────────────────────────────────────

def on_initialize(_ctx):
    if _state.get("hat") is None:
        print("[ERROR] on_initialize: hat not ready", file=sys.stderr)
        return -1
    print("[INFO] on_initialize: OK", file=sys.stderr)
    return 0


def on_reset(_ctx):
    return 0


def on_simulation_step(_ctx):
    hat = _state.get("hat")
    if hat is None or hat._hat is None:
        return -1

    _state["step_count"] += 1
    debug = _state.get("debug") and (_state["step_count"] % _DEBUG_INTERVAL == 0)

    # 1. コントローラーから旋回トルク指令を受信
    pwm = 0.0
    raw = hakopy.pdu_read(ROBOT_NAME, CH_TURRET_TORQUE, PDU_SIZE_TORQUE)
    if raw:
        try:
            pwm = float(pdu_to_py_Float64(bytearray(raw)).data)
            hat.motor_pwm_no_step(PORT_MOTOR, max(-1.0, min(1.0, pwm)))
        except Exception:
            pass

    # 2. MuJoCo を1ステップ進める（ペーシングはコントローラーの hakopy.usleep が担う）
    hat.sim_step_no_pace()
    if _state["nq"] > 0:
        _write_qpos(hat, _state["nq"])

    # 3. 距離センサー → Range PDU
    dist_m = float("inf")
    try:
        mm = hat.distance_read(PORT_DISTANCE)
        rng = Range()
        rng.radiation_type = 1
        rng.field_of_view  = 0.261799
        rng.min_range      = DIST_MIN_M
        rng.max_range      = DIST_MAX_M
        rng.range = float("inf") if (mm is None or mm >= DIST_INVALID_MM) else mm / 1000.0
        dist_m = rng.range
        hakopy.pdu_write(ROBOT_NAME, CH_RANGE, py_to_pdu_Range(rng), PDU_SIZE_RANGE)
    except Exception as e:
        print(f"[WARN] distance PDU write failed: {e}", file=sys.stderr)

    # 4. カラーセンサー → ColorRGBA PDU
    rgb = (0, 0, 0)
    try:
        r, g, b = hat.color_read_rgb(PORT_COLOR)
        rgb = (r, g, b)
        color = ColorRGBA()
        color.r = r / 255.0; color.g = g / 255.0; color.b = b / 255.0; color.a = 1.0
        hakopy.pdu_write(ROBOT_NAME, CH_COLOR_RGBA, py_to_pdu_ColorRGBA(color), PDU_SIZE_COLOR)
    except Exception as e:
        print(f"[WARN] color PDU write failed: {e}", file=sys.stderr)

    # 5. モーター角度 → Float64 PDU
    # motor_joint は qpos[4]（wall_a_x/y, wall_b_x/y の次）
    angle_deg_val = 0.0
    try:
        angle_deg_val = math.degrees(hat.sim_get_qpos(4))
        ang = Float64(); ang.data = angle_deg_val
        hakopy.pdu_write(ROBOT_NAME, CH_MOTOR_ANGLE, py_to_pdu_Float64(ang), PDU_SIZE_ANGLE)
    except Exception as e:
        print(f"[WARN] motor_angle PDU write failed: {e}", file=sys.stderr)

    # 6. フォースセンサー → Bool PDU
    try:
        fs = Bool(); fs.data = hat.force_is_pressed(PORT_FORCE)
        hakopy.pdu_write(ROBOT_NAME, CH_FORCE_SENSOR, py_to_pdu_Bool(fs), PDU_SIZE_FORCE)
    except Exception as e:
        print(f"[WARN] force_sensor PDU write failed: {e}", file=sys.stderr)

    if debug:
        print(
            f"[DBG step={_state['step_count']:6d}] "
            f"pwm={pwm:+.3f}  "
            f"dist={dist_m*1000:.0f}mm  "
            f"color=({rgb[0]:3d},{rgb[1]:3d},{rgb[2]:3d})  "
            f"angle={angle_deg_val:+.1f}deg",
            file=sys.stderr, flush=True,
        )

    return 0



# ─── メイン ──────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="sonar_radar Hakoniwa plant")
    parser.add_argument("--viewer", action="store_true", help="MuJoCo viewer を表示する（mjpython 必須）")
    parser.add_argument("--debug",  action="store_true", help="センサー値・PDU をデバッグ出力する")
    args = parser.parse_args()

    _state["debug"] = args.debug or bool(os.environ.get("SPIKEHAT_HAKO_DEBUG"))

    if not os.path.exists(_xml_path):
        print(f"[ERROR] XML not found: {_xml_path}", file=sys.stderr); sys.exit(1)
    if not os.path.exists(PDU_DEF_PATH):
        print(f"[ERROR] PDU def not found: {PDU_DEF_PATH}", file=sys.stderr); sys.exit(1)

    try:
        import mujoco
        mdl_tmp = mujoco.MjModel.from_xml_path(_xml_path)
        step_usec = int(mdl_tmp.opt.timestep * 1e6)
        _state["nq"] = int(mdl_tmp.nq)
    except Exception:
        step_usec = 2000

    cb = {
        "on_initialize":            on_initialize,
        "on_simulation_step":       on_simulation_step,
        "on_manual_timing_control": None,
        "on_reset":                 on_reset,
    }

    if not hakopy.conductor_start(step_usec, step_usec):
        print("[ERROR] hakopy.conductor_start failed", file=sys.stderr); sys.exit(1)
    print(f"[INFO] conductor started (step={step_usec}us)", file=sys.stderr)

    ret = hakopy.asset_register(
        ASSET_NAME, PDU_DEF_PATH, cb, step_usec, hakopy.HAKO_ASSET_MODEL_PLANT)
    if not ret:
        print("[ERROR] hakopy.asset_register failed", file=sys.stderr)
        hakopy.conductor_stop(); sys.exit(1)
    print(f"[INFO] '{ASSET_NAME}' 登録完了。hako-cmd start を待機中...", file=sys.stderr)

    try:
        hat = SpikeHat(xml_path=_xml_path)
        hat.port_config(PORT_MOTOR,    DEVICE_MOTOR_L)
        hat.port_config(PORT_COLOR,    DEVICE_COLOR)
        hat.port_config(PORT_DISTANCE, DEVICE_DISTANCE)
        hat.port_config(PORT_FORCE,    DEVICE_FORCE)
        _state["hat"] = hat
        print(f"[INFO] SpikeHat initialized: {_xml_path}", file=sys.stderr)
    except Exception as e:
        print(f"[ERROR] SpikeHat init failed: {e}", file=sys.stderr)
        hakopy.conductor_stop(); sys.exit(1)

    # --viewer: sonar_radar_viewer.py を別プロセス（mjpython）で起動する
    # on_simulation_step が /tmp/sonar_radar_qpos.bin へ qpos を書き込み、
    # viewer プロセスがそれを読んで表示する。GIL 問題は完全に回避できる。
    viewer_proc = None
    if args.viewer:
        viewer_script = os.path.join(_here, "sonar_radar_viewer.py")
        mjpython = "/opt/homebrew/bin/mjpython"
        if not os.path.exists(mjpython):
            print(f"[WARN] mjpython not found: {mjpython}", file=sys.stderr)
        elif not os.path.exists(viewer_script):
            print(f"[WARN] viewer script not found: {viewer_script}", file=sys.stderr)
        else:
            try:
                os.remove(_QPOS_FILE)   # 前回の残骸があると viewer が誤判定して即終了するため
            except FileNotFoundError:
                pass
            viewer_env = {**os.environ, "SPIKEHAT_SIM_XML": _xml_path}
            viewer_proc = _subprocess.Popen([mjpython, viewer_script], env=viewer_env)
            print(f"[INFO] viewer プロセス起動 (pid={viewer_proc.pid})", file=sys.stderr)

    # Ctrl-C ハンドラ（メインスレッドで hakopy.start() を呼ぶので受け取れる）
    def _on_signal(sig, _frame):
        print(f"[INFO] signal {sig} received", file=sys.stderr, flush=True)
        if viewer_proc and viewer_proc.poll() is None:
            viewer_proc.terminate()
        os._exit(0)

    signal.signal(signal.SIGINT,  _on_signal)
    signal.signal(signal.SIGTERM, _on_signal)

    # メインスレッドで hakopy.start() を呼ぶ
    # GIL を保持したままブロックするが、メインスレッドなので Ctrl-C（SIGINT）は届く
    hakopy.start()
    print("[INFO] hakopy.start returned", file=sys.stderr)

    # qpos の書き込みが止まると viewer プロセスは _IDLE_TIMEOUT 後に自動終了する
    if viewer_proc and viewer_proc.poll() is None:
        print("[INFO] viewer 終了待ち（最大8秒）...", file=sys.stderr)
        try:
            viewer_proc.wait(timeout=8.0)
        except _subprocess.TimeoutExpired:
            print("[INFO] viewer タイムアウト → 強制終了", file=sys.stderr)
            viewer_proc.terminate()

    hakopy.conductor_stop()
    print("[INFO] done.", file=sys.stderr)


if __name__ == "__main__":
    main()
