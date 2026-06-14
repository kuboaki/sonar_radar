#!/usr/bin/env python3
"""
sonar_radar_sim02.py — MuJoCo 仮想環境で sonar_radar02.py を実行するエントリポイント。

libspikehat_sim を使って sonar_radar02.py をそのまま実行する。

sonar_radar02.py と同じ引数（現在は引数なし）。

シミュレーション固有オプション:
  --viewer    ビューアを表示する（mjpython 推奨）
  --speed N   速度スケール（デフォルト 10）

【--viewer の動作】
  ┌─ mjpython で実行 ─────────────────────────────────────────────┐
  │  sonar_radar02.py 本体はメインスレッドでそのまま実行される      │
  │  （実機実行時と同じスレッド構成。SpikeHatのsignalハンドラも     │
  │   通常通り動作する）                                            │
  │  ビューアの表示更新は別スレッド（表示専用、launch_passive）で   │
  │   行い、メインスレッド側の実シミュレーション状態を反映する      │
  │  Control タブの press_ctrl を押すとスキャンが終了する           │
  │  コマンド例: mjpython sonar_radar_sim02.py --viewer           │
  └────────────────────────────────────────────────────────────────┘
  ┌─ python3 で実行 ──────────────────────────────────────────────┐
  │  スキャン完了後に launch で最終状態を表示（ブロッキング）       │
  │  コマンド例: python3 sonar_radar_sim02.py --viewer            │
  └────────────────────────────────────────────────────────────────┘

【環境変数】
  SPIKEHAT_SIM_XML : MuJoCo XML ファイルのパス
                     デフォルト: <このファイルのディレクトリ>/../mujoco_model/sonar_radar.xml
"""

import sys
import os
import types
import threading
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
_radar_path  = os.path.join(_here, "..", "raspi", "sonar_radar02.py")
_xml_default = os.path.join(_here, "..", "mujoco_model", "sonar_radar.xml")
_xml_path    = os.environ.get("SPIKEHAT_SIM_XML",
                               os.path.realpath(_xml_default))

# libspikehat_sim の Python バインディングを参照
sys.path.insert(0, _lib_sim_dir)

# ------------------------------------------------------------------ #
# spikehat モジュールの差し込み
# ------------------------------------------------------------------ #

def _inject_spikehat(xml_path, speed_scale, hat_holder=None):
    """
    libspikehat_sim の SpikeHat を spikehat.SpikeHat として差し込む。
    sonar_radar02.py はこの差し込みにより libspikehat_sim 上で動作する。

    hat_holder : list が渡された場合、生成した SpikeHat インスタンスを
                 追加する（ビューア側から ctrl 操作・状態取得するために使用）。
    """
    # --speed は SPIKEHAT_SIM_SPEED_SCALE 経由でシムに渡す
    # （環境変数が明示的に設定されている場合はそちらを優先する）
    if "SPIKEHAT_SIM_SPEED_SCALE" not in os.environ:
        os.environ["SPIKEHAT_SIM_SPEED_SCALE"] = str(speed_scale)

    import spikehat as _sh

    # SpikeHat に XML パスを設定するラッパー
    _OrigSpikeHat = _sh.SpikeHat

    class _SimSpikeHat(_OrigSpikeHat):
        def __init__(self):
            super().__init__(xml_path=xml_path)
            if hat_holder is not None:
                hat_holder.append(self)

    # spikehat モジュールを差し込む
    m = types.ModuleType("spikehat")
    m.SpikeHat        = _SimSpikeHat
    m.DEVICE_MOTOR_L  = _sh.DEVICE_MOTOR_L
    m.DEVICE_COLOR    = _sh.DEVICE_COLOR
    m.DEVICE_DISTANCE = _sh.DEVICE_DISTANCE
    m.DEVICE_FORCE    = _sh.DEVICE_FORCE
    sys.modules["spikehat"] = m


def _run_radar():
    """sonar_radar02.py を __main__ として実行する。"""
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
    import math
    import time as _time

    import mujoco
    import mujoco.viewer

    _mdl = mujoco.MjModel.from_xml_path(_xml_path)
    _dat = mujoco.MjData(_mdl)

    # 初期カメラ表示範囲: floor(2m四方)を含む全体の寸法ではなく、
    # レーダー本体(数cm)を基準にして拡大表示する
    _mdl.stat.center[:] = [0.0, -0.02, 0.03]
    _mdl.stat.extent = 0.12

    # mjpython かどうか判定
    _IS_MJPYTHON = getattr(mujoco.viewer, '_MJPYTHON', None) is not None

    # ---- mjpython: launch_passive でリアルタイム表示 ----
    if _IS_MJPYTHON:
        _motor_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "motor_joint")]
        _dome_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "dome_joint")]
        _press_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "press_ctrl")
        _press_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "press_slide")]
        _wall_x_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "wall_x_ctrl")
        _wall_y_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "wall_y_ctrl")
        _wall_x_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "wall_x")]
        _wall_y_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "wall_y")]

        # sonar_radar02.py 実行中に生成される SpikeHat インスタンスを受け取る
        _hat_holder = []
        _inject_spikehat(_xml_path, _sim_args.speed, _hat_holder)

        def _viewer_loop():
            """
            表示専用のループ（別スレッド）。
            メインスレッドで動いている実シミュレーション(libspikehat_sim)の
            状態を取得し、表示用の _mdl/_dat に反映して sync() するだけ。
            物理計算（mj_step）は行わない。
            """
            with mujoco.viewer.launch_passive(_mdl, _dat) as viewer:
                # floor(2m四方)込みの自動フィットだと小さく表示されるため、
                # レーダー本体(数cm)を基準にカメラを寄せる
                viewer.cam.lookat[:] = _mdl.stat.center
                viewer.cam.distance  = _mdl.stat.extent * 2.5
                viewer.cam.azimuth   = 180.0
                viewer.cam.elevation = -30.0

                while viewer.is_running():
                    if _hat_holder:
                        _hat = _hat_holder[0]

                        # Controlタブの press_ctrl / wall_x_ctrl / wall_y_ctrl を実シミュレーションへ転送
                        _hat.sim_set_ctrl(_press_aid, float(_dat.ctrl[_press_aid]))
                        _hat.sim_set_ctrl(_wall_x_aid, float(_dat.ctrl[_wall_x_aid]))
                        _hat.sim_set_ctrl(_wall_y_aid, float(_dat.ctrl[_wall_y_aid]))

                        # 実シミュレーションのモーター角度を表示用に反映
                        try:
                            _motor_rad = math.radians(_hat.motor_get_position(0))
                            _dat.qpos[_motor_qadr] = _motor_rad
                            _dat.qpos[_dome_qadr]  = -_motor_rad / 3.0
                            _dat.qvel[_motor_qadr] = 0.0
                            _dat.qvel[_dome_qadr]  = 0.0
                        except RuntimeError:
                            pass

                        # 終了スイッチ(press_body)の位置を表示用に反映
                        try:
                            _dat.qpos[_press_qadr] = _hat.sim_get_qpos(_press_qadr)
                            _dat.qvel[_press_qadr] = 0.0
                        except RuntimeError:
                            pass

                        # 壁(wall_body)の位置を表示用に反映
                        try:
                            _dat.qpos[_wall_x_qadr] = _hat.sim_get_qpos(_wall_x_qadr)
                            _dat.qvel[_wall_x_qadr] = 0.0
                            _dat.qpos[_wall_y_qadr] = _hat.sim_get_qpos(_wall_y_qadr)
                            _dat.qvel[_wall_y_qadr] = 0.0
                        except RuntimeError:
                            pass

                    # 表示専用: 拘束ソルバーを伴う mj_step / mj_forward は呼ばない
                    # （メインスレッド側 libspikehat_sim の mj_step と競合するため）
                    mujoco.mj_kinematics(_mdl, _dat)
                    mujoco.mj_comPos(_mdl, _dat)
                    viewer.sync()
                    _time.sleep(_mdl.opt.timestep)

        _viewer_thread = threading.Thread(target=_viewer_loop, daemon=True)
        _viewer_thread.start()

        print("[sim] スキャン中です。Controlタブの press_ctrl で終了スイッチを押せます。",
              file=sys.stderr)

        # sonar_radar02.py 本体はメインスレッドでそのまま実行する
        # （実機実行時と同じスレッド構成。SpikeHatのsignalハンドラも通常通り動作する）
        _run_radar()

        print("[sim] スキャン完了。ウィンドウを閉じると終了します。",
              file=sys.stderr)
        _viewer_thread.join()

    # ---- python3: スキャン完了後に静的表示 ----
    else:
        _inject_spikehat(_xml_path, _sim_args.speed)
        _run_radar()

        print("[sim] スキャン完了。ビューアで最終状態を表示します。",
              file=sys.stderr)
        print("[sim] リアルタイム表示には: mjpython sonar_radar_sim02.py --viewer",
              file=sys.stderr)
        mujoco.viewer.launch(_mdl, _dat)
