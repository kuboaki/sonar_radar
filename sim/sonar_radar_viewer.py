#!/usr/bin/env python3
"""
sonar_radar_viewer.py — MuJoCo viewer（plant とは別プロセスで動く）

sonar_radar_hako.py の --viewer オプションから mjpython で自動起動される。
/tmp/sonar_radar_qpos.bin に書かれた qpos をリアルタイムで読んで表示する。
シミュレーション終了後 _IDLE_TIMEOUT 秒更新がなければ自動的に閉じる。

単独起動:
  /opt/homebrew/bin/mjpython sonar_radar_viewer.py
"""

import sys
import os
import time
import struct

import mujoco
import mujoco.viewer

_here     = os.path.dirname(os.path.abspath(__file__))
_xml_path = os.environ.get(
    "SPIKEHAT_SIM_XML",
    os.path.realpath(os.path.join(_here, "..", "mujoco_model", "sonar_radar.xml")))

QPOS_FILE      = "/tmp/sonar_radar_qpos.bin"
PRESS_REQ_FILE = "/tmp/sonar_radar_press_req"
_IDLE_TIMEOUT  = 5.0   # この秒数ファイル更新がなければ自動終了

mdl = mujoco.MjModel.from_xml_path(_xml_path)
dat = mujoco.MjData(mdl)
nq  = mdl.nq
fmt = f"{nq}d"
sz  = nq * 8

last_mtime  = 0.0
last_update = time.monotonic()

print(f"[viewer] モデル: {_xml_path}  nq={nq}", file=sys.stderr, flush=True)
print(f"[viewer] qpos ファイル監視: {QPOS_FILE}", file=sys.stderr, flush=True)

def _key_callback(keycode):
    if keycode == ord(' '):
        try:
            with open(PRESS_REQ_FILE, "w") as f:
                f.write("1")
            print("[viewer] starter: 押下リクエスト送信", file=sys.stderr, flush=True)
        except Exception as e:
            print(f"[viewer] WARN: press req write failed: {e}", file=sys.stderr, flush=True)

with mujoco.viewer.launch_passive(mdl, dat, key_callback=_key_callback) as viewer:
    viewer.cam.lookat[:] = mdl.stat.center
    viewer.cam.distance  = mdl.stat.extent * 1.8
    viewer.cam.azimuth   = 155.0
    viewer.cam.elevation = -28.0
    print("[viewer] 起動完了", file=sys.stderr, flush=True)

    while viewer.is_running():
        try:
            mt = os.path.getmtime(QPOS_FILE)
            if mt != last_mtime:
                last_mtime  = mt
                last_update = time.monotonic()
                with open(QPOS_FILE, "rb") as f:
                    raw = f.read(sz)
                if len(raw) == sz:
                    qvals = struct.unpack(fmt, raw)
                    for i, v in enumerate(qvals):
                        dat.qpos[i] = v
                    mujoco.mj_kinematics(mdl, dat)
                    mujoco.mj_comPos(mdl, dat)
        except Exception as e:
            print(f"[viewer] WARN: {e}", file=sys.stderr, flush=True)

        # ファイル更新が止まったらシミュレーション終了と判断して自動終了
        if last_mtime > 0.0 and time.monotonic() - last_update > _IDLE_TIMEOUT:
            print("[viewer] シミュレーション終了を検出 → 自動終了", file=sys.stderr, flush=True)
            break

        viewer.sync()
        time.sleep(0.016)

print("[viewer] 終了", file=sys.stderr, flush=True)
