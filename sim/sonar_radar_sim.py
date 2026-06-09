#!/usr/bin/env python3
"""
sonar_radar_sim.py — MuJoCo 仮想環境でソナーレーダーを実行するエントリポイント。

sonar_radar.py と同じ引数:
  python3 sonar_radar_sim.py               # ±65° step スキャン
  python3 sonar_radar_sim.py --range 35
  python3 sonar_radar_sim.py --mode continuous

シミュレーション固有オプション:
  --viewer    ビューアを表示する
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
# パス設定・共通ヘルパー
# ------------------------------------------------------------------ #

_here       = os.path.dirname(os.path.abspath(__file__))
_sim_dir    = os.path.join(_here, "sim")
_radar_path = os.path.join(_here, "sonar_radar.py")

sys.path.insert(0, _sim_dir)
from sim_spikehat import SimSpikeHat as _Sim
from sim_spikehat import DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE


def _inject(hat_cls):
    """hat_cls を 'spikehat.SpikeHat' として sys.modules に登録する。"""
    m = types.ModuleType("spikehat")
    m.SpikeHat       = hat_cls
    m.DEVICE_MOTOR_L  = DEVICE_MOTOR_L
    m.DEVICE_COLOR    = DEVICE_COLOR
    m.DEVICE_DISTANCE = DEVICE_DISTANCE
    sys.modules["spikehat"] = m


def _run_radar():
    """sonar_radar.py を __main__ として実行する。"""
    spec = importlib.util.spec_from_file_location("__main__", _radar_path)
    mod  = importlib.util.module_from_spec(spec)
    mod.__name__ = "__main__"
    spec.loader.exec_module(mod)


# ================================================================== #
# viewer なし（通常モード）
# ================================================================== #

if not _sim_args.viewer:

    class _Hat(_Sim):
        def __init__(self):
            super().__init__(speed_scale=_sim_args.speed)

    _inject(_Hat)
    _run_radar()


# ================================================================== #
# viewer あり
# ================================================================== #

else:
    import mujoco
    import mujoco.viewer

    _xml  = os.path.join(_sim_dir, "sonar_radar.xml")
    _mdl  = mujoco.MjModel.from_xml_path(_xml)
    _dat  = mujoco.MjData(_mdl)

    # mjpython かどうか判定（mujoco.viewer._MJPYTHON が非 None なら mjpython）
    _IS_MJPYTHON = mujoco.viewer._MJPYTHON is not None

    # ---- mjpython: launch_passive でリアルタイム表示 ----
    if _IS_MJPYTHON:

        _hdl_ref = [None]   # viewer handle を with の外から参照するため

        class _Hat(_Sim):
            def __init__(self):
                super().__init__(speed_scale=_sim_args.speed,
                                 viewer=True,   # __enter__ で launch_passive を呼ぶ
                                 model=_mdl, data=_dat)

            def __enter__(self):
                result = super().__enter__()
                _hdl_ref[0] = self._viewer_hdl
                return result

        _inject(_Hat)
        _run_radar()

        # スキャン完了後、ウィンドウが閉じられるまで待機
        hdl = _hdl_ref[0]
        if hdl is not None and hdl.is_running():
            print("[sim] スキャン完了。ウィンドウを閉じると終了します。",
                  file=sys.stderr)
            while hdl.is_running():
                time.sleep(0.1)

    # ---- python3: スキャン完了後に静的表示 ----
    else:

        class _Hat(_Sim):
            def __init__(self):
                super().__init__(speed_scale=_sim_args.speed,
                                 viewer=False,  # スキャン中は viewer 不使用
                                 model=_mdl, data=_dat)

        _inject(_Hat)
        _run_radar()   # スキャン実行（JSON を stdout に出力）

        print("[sim] スキャン完了。ビューアで最終状態を表示します。",
              file=sys.stderr)
        print("[sim] リアルタイム表示には: mjpython sonar_radar_sim.py --viewer",
              file=sys.stderr)
        # launch はブロッキング。ウィンドウを閉じると終了する。
        mujoco.viewer.launch(_mdl, _dat)
