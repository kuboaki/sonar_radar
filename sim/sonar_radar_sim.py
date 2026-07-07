#!/usr/bin/env python3
"""
sonar_radar_sim.py — MuJoCo 仮想環境で SonarRadarSM を実行するエントリポイント。

libspikehat_sim を _SimSpikeHat でラップして SonarRadarSM に差し込み、
スタンドアロンの MuJoCo シミュレーションを実行する。

シミュレーション固有オプション:
  --viewer    ビューアを表示する（mjpython 推奨）
  --auto-start SEC   キャリブレーション後 SEC 秒でスキャン開始ボタンを自動注入
  --auto-stop  SEC   スキャン開始から SEC 秒後に停止ボタンを自動注入

【--viewer の動作】
  ┌─ mjpython で実行 ─────────────────────────────────────────────┐
  │  SonarRadarSM はメインスレッドで実行される                     │
  │  ビューアの表示更新は別スレッド（launch_passive）で行う         │
  │  Space キーで開始/終了トリガーを入力できる                      │
  │  コマンド例: mjpython sonar_radar_sim.py --viewer           │
  └────────────────────────────────────────────────────────────────┘
  ┌─ python3 で実行 ──────────────────────────────────────────────┐
  │  スキャン完了後に launch で最終状態を表示（ブロッキング）       │
  │  コマンド例: python3 sonar_radar_sim.py --viewer            │
  └────────────────────────────────────────────────────────────────┘

【環境変数】
  SPIKEHAT_SIM_XML : MuJoCo XML ファイルのパス
                     デフォルト: <このファイルのディレクトリ>/../mujoco_model/sonar_radar.xml
"""

import sys
import os
import types
import threading
import argparse
import json
import time as _time_mod

# ------------------------------------------------------------------ #
# シミュレーション固有引数を先にパース
# ------------------------------------------------------------------ #

_sim_parser = argparse.ArgumentParser(add_help=False)
_sim_parser.add_argument("--viewer", action="store_true")
_sim_parser.add_argument("--auto-start", type=float, default=None, metavar="SEC",
                         help="キャリブレーション後 SEC 秒でスキャン開始ボタンを自動注入")
_sim_parser.add_argument("--auto-stop", type=float, default=None, metavar="SEC",
                         help="スキャン開始から SEC 秒後に停止ボタンを自動注入")
_sim_args, _remaining = _sim_parser.parse_known_args()
sys.argv = [sys.argv[0]] + _remaining

# ------------------------------------------------------------------ #
# パス設定
# ------------------------------------------------------------------ #

_here        = os.path.dirname(os.path.abspath(__file__))
_raspi_dir   = os.path.join(_here, "..", "raspi")
_lib_sim_dir = os.path.join(_here, "libspikehat_sim", "python")
_xml_default = os.path.join(_here, "..", "mujoco_model", "sonar_radar.xml")
_xml_path    = os.environ.get("SPIKEHAT_SIM_XML", os.path.realpath(_xml_default))

sys.path.insert(0, _lib_sim_dir)

# ------------------------------------------------------------------ #
# _SimSpikeHat — libspikehat_sim の SpikeHat に sim 固有機能を追加
# ------------------------------------------------------------------ #

import spikehat as _sh

_AUTO_PRESS_DURATION = 0.15  # 押下を維持する wall-clock 秒


def _make_sim_hat(xml_path, hat_holder=None,
                  auto_start_sec=None, auto_stop_sec=None):
    """
    _SimSpikeHat インスタンスを生成して返す。

    hat_holder : list が渡された場合、生成したインスタンスを追加する
                 （ビューアスレッドからアクセスするために使用）。
    auto_start_sec / auto_stop_sec : wall-clock 秒で自動注入するスケジュール。
    """
    _OrigSpikeHat = _sh.SpikeHat

    # auto-press スケジュール（wall-clock 秒）
    _schedule  = [None, None]   # [start, stop]: 注入する絶対時刻
    _press_end = [None, None]   # [start, stop]: 押下期間の終了時刻
    # ビューア Space キーによる override
    _force_override = [0]       # 残り True 返却回数

    class _SimSpikeHat(_OrigSpikeHat):
        def __init__(self):
            super().__init__(xml_path=xml_path)
            if hat_holder is not None:
                hat_holder.append(self)

        def force_is_pressed(self, port):
            # Space キー override
            if hat_holder is not None and _force_override[0] > 0:
                _force_override[0] -= 1
                return True
            # auto-press スケジュール処理
            now = _time_mod.monotonic()
            for idx in (0, 1):
                if (_schedule[idx] is not None
                        and now >= _schedule[idx]
                        and _press_end[idx] is None):
                    _schedule[idx] = None
                    _press_end[idx] = now + _AUTO_PRESS_DURATION
                    label = "スタート" if idx == 0 else "ストップ"
                    print(f"[sim] auto-press: {label}ボタン注入 ({_AUTO_PRESS_DURATION*1000:.0f}ms)",
                          file=sys.stderr)
            for idx in (0, 1):
                if _press_end[idx] is not None:
                    if now < _press_end[idx]:
                        return True
                    _press_end[idx] = None
            return super().force_is_pressed(port)

        def close(self):
            # ビューアモードでは viewer スレッド終了まで実体を保持する
            if hat_holder is not None:
                return
            super().close()

        def _release_for_viewer(self):
            """viewer スレッド終了後に呼ぶ最終解放。"""
            super().close()

    hat = _SimSpikeHat()

    # auto-press スケジュールを wall-clock 絶対時刻で設定
    t0 = _time_mod.monotonic()
    if auto_start_sec is not None:
        _schedule[0] = t0 + auto_start_sec
        print(f"[sim] auto-start: {auto_start_sec:.1f}s 後にスタートボタンを注入", file=sys.stderr)
    if auto_stop_sec is not None:
        _schedule[1] = t0 + auto_stop_sec
        print(f"[sim] auto-stop: {auto_stop_sec:.1f}s 後にストップボタンを注入", file=sys.stderr)

    # Space キー用 override カウンタを hat に公開する（viewer loop から操作）
    hat._force_override = _force_override

    return hat


# ------------------------------------------------------------------ #
# SonarRadarSM を import（sonar_radar.py の module-level import 用スタブ差し込み）
# ------------------------------------------------------------------ #

_stub_spikehat = types.ModuleType("spikehat")
_stub_spikehat.SpikeHat        = object
_stub_spikehat.DEVICE_MOTOR_L  = _sh.DEVICE_MOTOR_L
_stub_spikehat.DEVICE_FORCE    = _sh.DEVICE_FORCE
_stub_spikehat.DEVICE_COLOR    = _sh.DEVICE_COLOR
_stub_spikehat.DEVICE_DISTANCE = _sh.DEVICE_DISTANCE
sys.modules.setdefault("spikehat", _stub_spikehat)

if _raspi_dir not in sys.path:
    sys.path.insert(0, _raspi_dir)

from sonar_radar import SonarRadarSM, SAMPLE_INTERVAL_S


# ------------------------------------------------------------------ #
# SM 実行
# ------------------------------------------------------------------ #

def _run_sm(hat):
    start_time = _time_mod.monotonic()
    sm = SonarRadarSM(clock=lambda: _time_mod.monotonic() - start_time)
    with hat:
        while not sm.is_terminated():
            sm.tick(hat)
            hat.sleep(SAMPLE_INTERVAL_S)
    print(json.dumps(sm.results, ensure_ascii=False, indent=2))


# ================================================================== #
# viewer なし（通常モード）
# ================================================================== #

if not _sim_args.viewer:
    _auto_stop_sec = None
    if _sim_args.auto_stop is not None:
        _base = _sim_args.auto_start if _sim_args.auto_start is not None else 0.0
        _auto_stop_sec = _base + _sim_args.auto_stop

    _hat = _make_sim_hat(_xml_path,
                         auto_start_sec=_sim_args.auto_start,
                         auto_stop_sec=_auto_stop_sec)
    _run_sm(_hat)


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

    _mdl.stat.center[:] = [0.0, 0.02, 0.04]
    _mdl.stat.extent = 0.22

    _IS_MJPYTHON = getattr(mujoco.viewer, '_MJPYTHON', None) is not None

    _WALL_STUD = 0.008

    _WALL_A_X_RANGE = (-0.08,  0.08)
    _WALL_A_Y_RANGE = (-0.080, 0.20)
    _WALL_B_X_RANGE = (-0.08,  0.136)
    _WALL_B_Y_RANGE = (-0.040, 0.25)

    def _snap_stud(val):
        return round(val / _WALL_STUD) * _WALL_STUD

    def _wall_step(aid, delta, ctrl_range):
        cur = _snap_stud(float(_dat.ctrl[aid]))
        _dat.ctrl[aid] = max(ctrl_range[0], min(ctrl_range[1], cur + delta))

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
        _button_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "button_slide")]
        _wall_a_x_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "wall_a_x_ctrl")
        _wall_a_y_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "wall_a_y_ctrl")
        _wall_b_x_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "wall_b_x_ctrl")
        _wall_b_y_aid = mujoco.mj_name2id(
            _mdl, mujoco.mjtObj.mjOBJ_ACTUATOR, "wall_b_y_ctrl")
        _wall_a_x_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "wall_a_x")]
        _wall_a_y_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "wall_a_y")]
        _wall_b_x_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "wall_b_x")]
        _wall_b_y_qadr = _mdl.jnt_qposadr[
            mujoco.mj_name2id(_mdl, mujoco.mjtObj.mjOBJ_JOINT, "wall_b_y")]

        _hat_holder = []
        _auto_stop_sec = None
        if _sim_args.auto_stop is not None:
            _base = _sim_args.auto_start if _sim_args.auto_start is not None else 0.0
            _auto_stop_sec = _base + _sim_args.auto_stop

        _hat = _make_sim_hat(_xml_path, _hat_holder,
                             auto_start_sec=_sim_args.auto_start,
                             auto_stop_sec=_auto_stop_sec)

        _KEY_RIGHT = 262
        _KEY_LEFT  = 263
        _KEY_DOWN  = 264
        _KEY_UP    = 265
        _selected_wall = [0]
        _PRESS_CALLS  = 10
        _PRESS_FORCE  = 3.0
        _PRESS_FRAMES = int(0.15 / _mdl.opt.timestep)
        _press_pulse  = [0]

        def _key_callback(keycode):
            if keycode == ord(' '):
                _press_pulse[0]           = _PRESS_FRAMES
                _hat._force_override[0]   = _PRESS_CALLS
                print("[sim] starter: 押下パルス送信", file=sys.stderr)
                return
            s = _WALL_STUD
            if keycode == ord('1'):
                _selected_wall[0] = 0
                print("[sim] 黄色壁(wall_a)を選択", file=sys.stderr)
            elif keycode == ord('2'):
                _selected_wall[0] = 1
                print("[sim] 黒壁(wall_b)を選択", file=sys.stderr)
            elif _selected_wall[0] == 0:
                if   keycode == _KEY_RIGHT: _wall_step(_wall_a_x_aid, +s, _WALL_A_X_RANGE)
                elif keycode == _KEY_LEFT:  _wall_step(_wall_a_x_aid, -s, _WALL_A_X_RANGE)
                elif keycode == _KEY_UP:    _wall_step(_wall_a_y_aid, +s, _WALL_A_Y_RANGE)
                elif keycode == _KEY_DOWN:  _wall_step(_wall_a_y_aid, -s, _WALL_A_Y_RANGE)
            else:
                if   keycode == _KEY_RIGHT: _wall_step(_wall_b_x_aid, +s, _WALL_B_X_RANGE)
                elif keycode == _KEY_LEFT:  _wall_step(_wall_b_x_aid, -s, _WALL_B_X_RANGE)
                elif keycode == _KEY_UP:    _wall_step(_wall_b_y_aid, +s, _WALL_B_Y_RANGE)
                elif keycode == _KEY_DOWN:  _wall_step(_wall_b_y_aid, -s, _WALL_B_Y_RANGE)

        def _viewer_loop():
            with mujoco.viewer.launch_passive(
                    _mdl, _dat,
                    key_callback=_key_callback,
                    show_left_ui=False) as viewer:
                viewer.cam.lookat[:] = _mdl.stat.center
                viewer.cam.distance  = _mdl.stat.extent * 1.8
                viewer.cam.azimuth   = 155.0
                viewer.cam.elevation = -28.0

                while viewer.is_running():
                    if _hat_holder:
                        _h = _hat_holder[0]

                        if _press_pulse[0] > 0:
                            _press_pulse[0] -= 1
                            _dat.ctrl[_press_aid] = _PRESS_FORCE
                        else:
                            _dat.ctrl[_press_aid] = 0.0

                        _h.sim_set_ctrl(_press_aid, float(_dat.ctrl[_press_aid]))
                        for _aid in (_wall_a_x_aid, _wall_a_y_aid,
                                     _wall_b_x_aid, _wall_b_y_aid):
                            _sv = _snap_stud(_dat.ctrl[_aid])
                            _dat.ctrl[_aid] = _sv
                            _h.sim_set_ctrl(_aid, _sv)

                        try:
                            _motor_rad = math.radians(_h.motor_get_position(0))
                            _dat.qpos[_motor_qadr] = _motor_rad
                            _dat.qpos[_dome_qadr]  = +_motor_rad / 3.0
                            _dat.qvel[_motor_qadr] = 0.0
                            _dat.qvel[_dome_qadr]  = 0.0
                        except RuntimeError:
                            pass

                        try:
                            _dat.qpos[_press_qadr]  = _h.sim_get_qpos(_press_qadr)
                            _dat.qvel[_press_qadr]  = 0.0
                            _dat.qpos[_button_qadr] = _h.sim_get_qpos(_button_qadr)
                            _dat.qvel[_button_qadr] = 0.0
                        except RuntimeError:
                            pass

                        try:
                            for _qadr in (_wall_a_x_qadr, _wall_a_y_qadr,
                                          _wall_b_x_qadr, _wall_b_y_qadr):
                                _dat.qpos[_qadr] = _h.sim_get_qpos(_qadr)
                                _dat.qvel[_qadr] = 0.0
                        except RuntimeError:
                            pass

                    mujoco.mj_kinematics(_mdl, _dat)
                    mujoco.mj_comPos(_mdl, _dat)
                    viewer.sync()
                    _time.sleep(_mdl.opt.timestep)

        _viewer_thread = threading.Thread(target=_viewer_loop, daemon=True)
        _viewer_thread.start()

        print("[sim] Space キーで開始/終了トリガーを入力してください。", file=sys.stderr)
        print("[sim] 壁の移動: 1=黄色壁選択 / 2=黒壁選択  矢印キー(←→X / ↑↓Y)で移動",
              file=sys.stderr)

        _run_sm(_hat)

        print("[sim] スキャン完了。ウィンドウを閉じると終了します。", file=sys.stderr)
        _viewer_thread.join()

        if _hat_holder:
            _hat_holder[0]._release_for_viewer()

    # ---- python3: スキャン完了後に静的表示 ----
    else:
        _auto_stop_sec = None
        if _sim_args.auto_stop is not None:
            _base = _sim_args.auto_start if _sim_args.auto_start is not None else 0.0
            _auto_stop_sec = _base + _sim_args.auto_stop

        _hat = _make_sim_hat(_xml_path,
                             auto_start_sec=_sim_args.auto_start,
                             auto_stop_sec=_auto_stop_sec)
        _run_sm(_hat)

        print("[sim] スキャン完了。ビューアで最終状態を表示します。", file=sys.stderr)
        print("[sim] リアルタイム表示には: mjpython sonar_radar_sim.py --viewer",
              file=sys.stderr)
        mujoco.viewer.launch(_mdl, _dat)
