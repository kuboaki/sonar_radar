#!/usr/bin/env python3
"""
sonar_radar_sim.py — MuJoCo 仮想環境で sonar_radar.py を実行するエントリポイント。

libspikehat_sim を使って sonar_radar.py をそのまま実行する。

sonar_radar.py と同じ引数（現在は引数なし）。

シミュレーション固有オプション:
  --viewer    ビューアを表示する（mjpython 推奨）
  --speed N   速度スケール（デフォルト 1 = 実時間）

【--viewer の動作】
  ┌─ mjpython で実行 ─────────────────────────────────────────────┐
  │  sonar_radar.py 本体はメインスレッドでそのまま実行される      │
  │  （実機実行時と同じスレッド構成。SpikeHatのsignalハンドラも     │
  │   通常通り動作する）                                            │
  │  ビューアの表示更新は別スレッド（表示専用、launch_passive）で   │
  │   行い、メインスレッド側の実シミュレーション状態を反映する      │
  │  Controlタブのpress_ctrlを押して離すとスキャン開始/停止できる    │
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
import importlib.util
import argparse

# ------------------------------------------------------------------ #
# シミュレーション固有引数を先にパース
# ------------------------------------------------------------------ #

_sim_parser = argparse.ArgumentParser(add_help=False)
_sim_parser.add_argument("--viewer", action="store_true")
_sim_parser.add_argument("--speed", type=float, default=1.0, metavar="N")
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

def _inject_spikehat(xml_path, speed_scale, hat_holder=None):
    """
    libspikehat_sim の SpikeHat を spikehat.SpikeHat として差し込む。
    sonar_radar.py はこの差し込みにより libspikehat_sim 上で動作する。

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

        def force_is_pressed(self, port):
            if hat_holder is not None and _force_override[0] > 0:
                _force_override[0] -= 1
                return True
            return super().force_is_pressed(port)

        def close(self):
            # ビューアモードでは、sonar_radar.py の with ブロックを抜けた後も
            # _viewer_loop（別スレッド）が同じ実体(mjModel/mjData/sim構造体)を
            # 参照し続ける。そのため close() では実体を解放せず、
            # 終了スイッチ操作によるモーター停止状態のまま維持する。
            # 実体の解放は _release_for_viewer() で、_viewer_thread.join() 後に行う。
            if hat_holder is not None:
                return
            super().close()

        def _release_for_viewer(self):
            """ビューアスレッド終了後に呼ぶ、実体(mjModel/mjData/sim構造体)の最終解放。"""
            super().close()

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
    import math
    import time as _time

    import mujoco
    import mujoco.viewer

    _mdl = mujoco.MjModel.from_xml_path(_xml_path)
    _dat = mujoco.MjData(_mdl)

    # 初期カメラ表示範囲: システム全体（台座・ドーム・スターター・壁）が収まるよう設定
    # X: -0.10〜0.12, Y: -0.09(壁前方)〜+0.12(starter後方), Z: 0〜0.08
    # 中心を Y=0.02（スターターと壁の中間付近）、Z=0.04（高さ中心）に設定
    _mdl.stat.center[:] = [0.0, 0.02, 0.04]
    _mdl.stat.extent = 0.22

    # mjpython かどうか判定
    _IS_MJPYTHON = getattr(mujoco.viewer, '_MJPYTHON', None) is not None

    _WALL_STUD = 0.008  # 壁移動の最小単位: 1スタッドピッチ = 8mm

    # 壁のctrlrange（sonar_radar.xmlのctrlrangeと一致させること）
    _WALL_A_X_RANGE = (-0.08,  0.08)
    _WALL_A_Y_RANGE = (-0.080, 0.20)
    _WALL_B_X_RANGE = (-0.08,  0.136)
    _WALL_B_Y_RANGE = (-0.040, 0.25)

    def _snap_stud(val):
        """壁のctrl値をスタッドピッチ単位に丸める"""
        return round(val / _WALL_STUD) * _WALL_STUD

    def _wall_step(aid, delta, ctrl_range):
        """壁のctrl値を1スタッド単位で増減しclampする（key_callbackから呼ぶ）"""
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

        # sonar_radar.py 実行中に生成される SpikeHat インスタンスを受け取る
        _hat_holder = []
        _inject_spikehat(_xml_path, _sim_args.speed, _hat_holder)

        # GLFW キーコード
        _KEY_RIGHT = 262
        _KEY_LEFT  = 263
        _KEY_DOWN  = 264
        _KEY_UP    = 265

        # 操作対象の壁: 0=黄色(wall_a), 1=黒(wall_b)
        _selected_wall = [0]

        # スペースキーによる starter パルス制御
        # force_is_pressed を直接オーバーライドして押下→離す を再現する。
        # _force_override > 0: True を返す(押下中), 0: 通常の物理ベース判定
        # MIN_PRESS_S(0.1s) を超えるよう、force_is_pressed を 10回以上 True にする
        # （polling 周期 20ms × 10 = 200ms）
        _PRESS_CALLS  = 10    # True を返す force_is_pressed 呼び出し回数
        _force_override = [0]  # 残り True 返却回数
        # 物理アクチュエーターへのパルスも維持（視覚確認用）
        _PRESS_FORCE  = 3.0
        _PRESS_FRAMES = int(0.15 / _mdl.opt.timestep)
        _press_pulse  = [0]

        def _key_callback(keycode):
            """
            キーボード操作:
              Space     : starter を「押して離す」（スキャン開始/終了トリガー）
              1         : 黄色壁(wall_a)を選択
              2         : 黒壁(wall_b)を選択
              矢印キー ← → : 選択中の壁 X方向
              矢印キー ↑ ↓ : 選択中の壁 Y方向（↑=壁方向、↓=退避方向）
            """
            if keycode == ord(' '):
                _press_pulse[0]   = _PRESS_FRAMES
                _force_override[0] = _PRESS_CALLS
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
            """
            表示専用のループ（別スレッド）。
            メインスレッドで動いている実シミュレーション(libspikehat_sim)の
            状態を取得し、表示用の _mdl/_dat に反映して sync() するだけ。
            物理計算（mj_step）は行わない。
            """
            with mujoco.viewer.launch_passive(
                    _mdl, _dat,
                    key_callback=_key_callback,
                    show_left_ui=False) as viewer:
                # システム全体が見えるカメラ設定
                viewer.cam.lookat[:] = _mdl.stat.center
                viewer.cam.distance  = _mdl.stat.extent * 1.8
                viewer.cam.azimuth   = 155.0   # 左前方から: 壁(左)・ドーム(中央奥)・starter(右)
                viewer.cam.elevation = -28.0   # やや急な俯瞰

                while viewer.is_running():
                    if _hat_holder:
                        _hat = _hat_holder[0]

                        # starter パルス処理（Space キー）
                        # パルス中は PRESS_FORCE を維持、終了後に 0 に戻す
                        if _press_pulse[0] > 0:
                            _press_pulse[0] -= 1
                            _dat.ctrl[_press_aid] = _PRESS_FORCE
                        else:
                            _dat.ctrl[_press_aid] = 0.0

                        # Controlタブの各ctrlを実シミュレーションへ転送
                        # 壁はスタッドピッチ単位にスナップしてからControlタブとシムの両方に反映
                        _hat.sim_set_ctrl(_press_aid, float(_dat.ctrl[_press_aid]))
                        for _aid in (_wall_a_x_aid, _wall_a_y_aid, _wall_b_x_aid, _wall_b_y_aid):
                            _sv = _snap_stud(_dat.ctrl[_aid])
                            _dat.ctrl[_aid] = _sv
                            _hat.sim_set_ctrl(_aid, _sv)

                        # 実シミュレーションのモーター角度を表示用に反映
                        try:
                            _motor_rad = math.radians(_hat.motor_get_position(0))
                            _dat.qpos[_motor_qadr] = _motor_rad
                            _dat.qpos[_dome_qadr]  = +_motor_rad / 3.0
                            _dat.qvel[_motor_qadr] = 0.0
                            _dat.qvel[_dome_qadr]  = 0.0
                        except RuntimeError:
                            pass

                        # starter の press_block と button 位置を表示用に反映
                        try:
                            _dat.qpos[_press_qadr]   = _hat.sim_get_qpos(_press_qadr)
                            _dat.qvel[_press_qadr]   = 0.0
                            _dat.qpos[_button_qadr]  = _hat.sim_get_qpos(_button_qadr)
                            _dat.qvel[_button_qadr]  = 0.0
                        except RuntimeError:
                            pass

                        # 壁A/Bの位置を表示用に反映
                        try:
                            _dat.qpos[_wall_a_x_qadr] = _hat.sim_get_qpos(_wall_a_x_qadr)
                            _dat.qvel[_wall_a_x_qadr] = 0.0
                            _dat.qpos[_wall_a_y_qadr] = _hat.sim_get_qpos(_wall_a_y_qadr)
                            _dat.qvel[_wall_a_y_qadr] = 0.0
                            _dat.qpos[_wall_b_x_qadr] = _hat.sim_get_qpos(_wall_b_x_qadr)
                            _dat.qvel[_wall_b_x_qadr] = 0.0
                            _dat.qpos[_wall_b_y_qadr] = _hat.sim_get_qpos(_wall_b_y_qadr)
                            _dat.qvel[_wall_b_y_qadr] = 0.0
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

        print("[sim] Controlタブの press_ctrl を 0→10→0 と動かして開始トリガーを入力してください。",
              file=sys.stderr)
        print("[sim] 壁の移動: 1=黄色壁選択 / 2=黒壁選択  矢印キー(←→X / ↑↓Y)で移動",
              file=sys.stderr)

        # sonar_radar.py 本体はメインスレッドでそのまま実行する
        # （実機実行時と同じスレッド構成。SpikeHatのsignalハンドラも通常通り動作する）
        _run_radar()

        print("[sim] スキャン完了。ウィンドウを閉じると終了します。",
              file=sys.stderr)
        _viewer_thread.join()

        # _viewer_loop が同じ実体を参照し終えた後に、最終的に解放する
        # （sonar_radar.py の close() ではこの解放を行っていない）
        if _hat_holder:
            _hat_holder[0]._release_for_viewer()

    # ---- python3: スキャン完了後に静的表示 ----
    else:
        _inject_spikehat(_xml_path, _sim_args.speed)
        _run_radar()

        print("[sim] スキャン完了。ビューアで最終状態を表示します。",
              file=sys.stderr)
        print("[sim] リアルタイム表示には: mjpython sonar_radar_sim.py --viewer",
              file=sys.stderr)
        mujoco.viewer.launch(_mdl, _dat)
