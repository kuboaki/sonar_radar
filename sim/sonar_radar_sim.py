#!/usr/bin/env python3
"""
sonar_radar_sim.py — MuJoCo 仮想環境でソナーレーダーを実行するエントリポイント。

libspikehat_sim を使って sonar_radar.py をそのまま実行する。

sonar_radar.py と同じ引数:
  python3 sonar_radar_sim.py               # ±65° step スキャン
  python3 sonar_radar_sim.py --range 35
  python3 sonar_radar_sim.py --mode continuous

シミュレーション固有オプション:
  --viewer    ビューアを表示する（mjpython 推奨）
  --speed N   速度スケール（デフォルト 10）

【--viewer の動作】
  ┌─ mjpython で実行 ─────────────────────────────────────────────┐
  │  launch_passive でリアルタイム表示（バックグラウンド viewer）  │
  │  スキャン中にアームが動く様子が見える                           │
  │  コマンド例: mjpython sonar_radar_sim.py --viewer             │
  └────────────────────────────────────────────────────────────────┘
  ┌─ python3 で実行 ──────────────────────────────────────────────┐
  │  スキャン完了後に launch で最終状態を表示（ブロッキング）       │
  │  コマンド例: python3 sonar_radar_sim.py --viewer              │
  └────────────────────────────────────────────────────────────────┘

【環境変数】
  SPIKEHAT_SIM_XML : MuJoCo XML ファイルのパス
                     デフォルト: <このファイルのディレクトリ>/../mujoco_model/sonar_radar.xml
"""

import sys
import os
import time
import types
import importlib.util
import argparse

# ------------------------------------------------------------------ #
# シミュレーション固有引数を先にパース
# ------------------------------------------------------------------ #

_sim_parser = argparse.ArgumentParser(add_help=False)
_sim_parser.add_argument("--viewer", action="store_true")
_sim_parser.add_argument("--speed", type=float, default=10.0, metavar="N")
_sim_args, _remaining = _sim_parser.parse_known_args()
sys.argv = [sys.argv[0]] + _remaining

# ------------------------------------------------------------------ #
# パス設定
# ------------------------------------------------------------------ #

_here        = os.path.dirname(os.path.abspath(__file__))
_lib_sim_dir = os.path.join(_here, "libspikehat_sim", "python")
_radar_path  = os.path.join(_here, "..", "raspi", "sonar_radar.py")
_xml_default = os.path.join(_here, "..", "mujoco_model", "sonar_radar.xml")
_xml_path    = os.environ.get("SPIKEHAT_SIM_XML",
                               os.path.realpath(_xml_default))

# libspikehat_sim の Python バインディングを参照
sys.path.insert(0, _lib_sim_dir)

# ------------------------------------------------------------------ #
# spikehat モジュールの差し込み
# ------------------------------------------------------------------ #

def _inject_spikehat(xml_path, speed_scale, viewer=False, model=None, data=None):
    """
    libspikehat_sim の SpikeHat を spikehat.SpikeHat として差し込む。
    sonar_radar.py はこの差し込みにより libspikehat_sim 上で動作する。
    """
    import spikehat as _sh

    # SpikeHat に XML パスと speed_scale を設定するラッパー
    _OrigSpikeHat = _sh.SpikeHat

    class _SimSpikeHat(_OrigSpikeHat):
        def __init__(self):
            super().__init__(xml_path=xml_path)
            # speed_scale は C 側の speed_scale として設定済み

    # spikehat モジュールを差し込む
    m = types.ModuleType("spikehat")
    m.SpikeHat        = _SimSpikeHat
    m.DEVICE_MOTOR_L  = _sh.DEVICE_MOTOR_L
    m.DEVICE_COLOR    = _sh.DEVICE_COLOR
    m.DEVICE_DISTANCE = _sh.DEVICE_DISTANCE
    m.DEVICE_FORCE    = _sh.DEVICE_FORCE
    sys.modules["spikehat"] = m


def _run_radar():
    """sonar_radar.py を __main__ として実行する。"""
    spec = importlib.util.spec_from_file_location(
        "__main__", os.path.realpath(_radar_path))
    mod  = importlib.util.module_from_spec(spec)
    mod.__name__ = "__main__"
    spec.loader.exec_module(mod)


# ================================================================== #
# viewer なし（通常モード）
# ================================================================== #

if not _sim_args.viewer:
    _inject_spikehat(_xml_path, _sim_args.speed)
    _run_radar()


# ================================================================== #
# viewer あり
# ================================================================== #

else:
    import mujoco
    import mujoco.viewer

    _mdl = mujoco.MjModel.from_xml_path(_xml_path)
    _dat = mujoco.MjData(_mdl)

    # mjpython かどうか判定
    _IS_MJPYTHON = getattr(mujoco.viewer, '_MJPYTHON', None) is not None

    # ---- mjpython: launch_passive でリアルタイム表示 ----
    if _IS_MJPYTHON:
        # ビューアを先に起動してからスキャン実行
        with mujoco.viewer.launch_passive(_mdl, _dat) as _viewer_hdl:
            _inject_spikehat(_xml_path, _sim_args.speed)
            _run_radar()

            # スキャン完了後、ウィンドウが閉じられるまで待機
            if _viewer_hdl.is_running():
                print("[sim] スキャン完了。ウィンドウを閉じると終了します。",
                      file=sys.stderr)
                while _viewer_hdl.is_running():
                    mujoco.mj_step(_mdl, _dat)
                    _viewer_hdl.sync()

    # ---- python3: スキャン完了後に静的表示 ----
    else:
        _inject_spikehat(_xml_path, _sim_args.speed)
        _run_radar()

        print("[sim] スキャン完了。ビューアで最終状態を表示します。",
              file=sys.stderr)
        print("[sim] リアルタイム表示には: mjpython sonar_radar_sim.py --viewer",
              file=sys.stderr)
        mujoco.viewer.launch(_mdl, _dat)
