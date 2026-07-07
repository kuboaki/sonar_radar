#!/usr/bin/env python3
"""
test_scan_compare.py - 実機/シミュレーター共通スキャン比較テスト

実機（Raspberry Pi）とシミュレーター（Hakoniwa PDU）で同一ロジックを実行し、
カラーセンサーの赤・青・その他の検出数を比較するためのデータを収集する。

動作:
  1. モーターを PWM で旋回開始
  2. カラーセンサーで赤・青マーカーを検出するたびに旋回方向を反転
  3. MAX_REVERSALS 回反転したらスキャン終了
  4. 結果を JSON で標準出力（実機）またはファイルに出力

使用方法:
  実機（Raspberry Pi）:
    python3 test_scan_compare.py [出力ファイル.json]

  シミュレーター（Hakoniwa）:
    bash run-hakopy.bash examples/spikehat/test_scan_compare.py [出力ファイル.json]
"""

import sys
import json
import time

# --- 定数 ---
PORT_MOTOR    = 0
PORT_FORCE    = 1
PORT_COLOR    = 2
PORT_DISTANCE = 3

SCAN_PWM        = 0.15   # 旋回速度
MAX_REVERSALS   = 6      # この回数反転したら終了（赤3往復+青3往復程度）
SAMPLE_INTERVAL = 0.1    # サンプリング間隔（秒）
DIST_INVALID    = 2000
DIST_OFFSET_MM  = 25

# HSV 判定閾値
# 注意: 実機（SPIKE Prime）は内蔵 LED で照らすため V が高くなる。
#       シムの geom rgba は視覚的な暗さ（青 V≈51）のため BLUE_VAL_MIN を低く設定する。
#       この差は実機/シム間の校正差として記録する。
RED_SAT_MIN  = 40
RED_VAL_MIN  = 40
BLUE_HUE_LO  = 210
BLUE_HUE_HI  = 270
BLUE_SAT_MIN = 40    # 実機相当: 58（sonar_radar.py の 580 を 0-100 スケールに換算）
BLUE_VAL_MIN = 30    # 実機相当: 100。シムの青 geom V≈51 に合わせて緩和


def rgb_to_hsv(r, g, b):
    """RGB (0-255) → HSV (H:0-360, S:0-100, V:0-100)"""
    r_, g_, b_ = r / 255.0, g / 255.0, b / 255.0
    cmax = max(r_, g_, b_)
    cmin = min(r_, g_, b_)
    diff = cmax - cmin

    if diff == 0:
        h = 0
    elif cmax == r_:
        h = int(60 * ((g_ - b_) / diff) % 360)
    elif cmax == g_:
        h = int(60 * ((b_ - r_) / diff) + 120)
    else:
        h = int(60 * ((r_ - g_) / diff) + 240)
    if h < 0:
        h += 360

    s = 0 if cmax == 0 else int((diff / cmax) * 100)
    v = int(cmax * 100)
    return h, s, v


def is_red(h, s, v):
    if s < RED_SAT_MIN or v < RED_VAL_MIN:
        return False
    return h >= 340 or h <= 20


def is_blue(h, s, v):
    if s < BLUE_SAT_MIN or v < BLUE_VAL_MIN:
        return False
    return BLUE_HUE_LO <= h <= BLUE_HUE_HI


def classify(r, g, b):
    """RGB → 'red' / 'blue' / 'floor' / 'none'"""
    if r == 0 and g == 0 and b == 0:
        return 'none'
    h, s, v = rgb_to_hsv(r, g, b)
    if is_red(h, s, v):
        return 'red'
    if is_blue(h, s, v):
        return 'blue'
    return 'floor'


def do_scan(hat):
    """スキャン実行。サンプルリストを返す。"""
    samples   = []
    pwm       = SCAN_PWM
    reversals = 0
    on_marker = False

    hat.motor_pwm(PORT_MOTOR, pwm)
    print(f"スキャン開始 (PWM={pwm}, 最大反転={MAX_REVERSALS}回)", file=sys.stderr)

    while reversals < MAX_REVERSALS:
        # カラー読取・分類
        try:
            r, g, b = hat.color_read_rgb(PORT_COLOR)
        except RuntimeError:
            r, g, b = 0, 0, 0

        label = classify(r, g, b)
        marker = (label == 'red' or label == 'blue')

        # マーカー検出エッジで反転
        if marker and not on_marker:
            pwm = -pwm
            hat.motor_pwm(PORT_MOTOR, pwm)
            reversals += 1
            print(f"  [{label}] 反転 {reversals}/{MAX_REVERSALS}", file=sys.stderr)
        on_marker = marker

        # 距離読取
        try:
            mm = hat.distance_read(PORT_DISTANCE)
            if mm == DIST_INVALID:
                mm = None
            elif mm is not None:
                mm = mm + DIST_OFFSET_MM
        except RuntimeError:
            mm = None

        samples.append({
            "step":        len(samples),
            "r": r, "g": g, "b": b,
            "label":       label,
            "distance_mm": mm,
        })

        hat.sleep(SAMPLE_INTERVAL)

    hat.motor_stop(PORT_MOTOR)
    hat.sleep(0.3)
    print(f"スキャン完了: {len(samples)} サンプル", file=sys.stderr)
    return samples


def make_summary(samples):
    counts = {"red": 0, "blue": 0, "floor": 0, "none": 0}
    for s in samples:
        counts[s["label"]] += 1
    dists = [s["distance_mm"] for s in samples if s["distance_mm"] is not None]
    return {
        "total_samples": len(samples),
        "red_hits":      counts["red"],
        "blue_hits":     counts["blue"],
        "floor_hits":    counts["floor"],
        "no_hit":        counts["none"],
        "dist_valid":    len(dists),
        "dist_min_mm":   min(dists) if dists else None,
        "dist_max_mm":   max(dists) if dists else None,
    }


# ─── 実機モード ────────────────────────────────────────────────────────────

def run_real(output_path):
    try:
        from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE
    except ImportError:
        import os
        sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'libspikehat', 'python'))
        from spikehat import SpikeHat, DEVICE_MOTOR_L, DEVICE_COLOR, DEVICE_DISTANCE

    with SpikeHat() as hat:
        hat.port_config(PORT_MOTOR,    DEVICE_MOTOR_L)
        hat.port_config(PORT_COLOR,    DEVICE_COLOR)
        hat.port_config(PORT_DISTANCE, DEVICE_DISTANCE)
        hat.sleep(1.0)

        samples = do_scan(hat)

    result = {"summary": make_summary(samples), "samples": samples}
    out = json.dumps(result, ensure_ascii=False, indent=2)
    if output_path:
        with open(output_path, "w") as f:
            f.write(out)
        print(f"→ {output_path} に保存", file=sys.stderr)
    else:
        print(out)


# ─── Hakoniwa モード ───────────────────────────────────────────────────────

def run_hakoniwa(output_path):
    import ctypes
    import hakopy

    LIB_PATH    = "src/cmake-build/robots/spikehat/libspikehat_pdu.dylib"
    CONFIG_PATH = "config/robots/spikehat/pdu_spikehat.json"
    PDU_DEF_PATH = "config/sonar-radar-pdudef-compact.json"
    ASSET_NAME  = "SonarRadarController"
    STEP_USEC   = 100_000

    lib = ctypes.CDLL(LIB_PATH)
    lib.spikehat_open.restype  = ctypes.c_void_p
    lib.spikehat_open.argtypes = [ctypes.c_char_p]
    lib.spikehat_close.argtypes  = [ctypes.c_void_p]
    lib.spikehat_motor_pwm.argtypes   = [ctypes.c_void_p, ctypes.c_int, ctypes.c_float]
    lib.spikehat_motor_stop.argtypes  = [ctypes.c_void_p, ctypes.c_int]
    lib.spikehat_distance_read.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int)]
    lib.spikehat_color_read_rgb.argtypes = [
        ctypes.c_void_p, ctypes.c_int,
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int),
        ctypes.POINTER(ctypes.c_int)]

    # SpikeHat ライクなラッパー（Hakoniwa 版）
    class HakoniwaHat:
        def __init__(self, handle):
            self._h = handle

        def motor_pwm(self, port, pwm):
            lib.spikehat_motor_pwm(self._h, port, ctypes.c_float(pwm))

        def motor_stop(self, port):
            lib.spikehat_motor_stop(self._h, port)

        def color_read_rgb(self, port):
            r, g, b = ctypes.c_int(0), ctypes.c_int(0), ctypes.c_int(0)
            lib.spikehat_color_read_rgb(self._h, port,
                ctypes.byref(r), ctypes.byref(g), ctypes.byref(b))
            return r.value, g.value, b.value

        def distance_read(self, port):
            mm = ctypes.c_int(0)
            lib.spikehat_distance_read(self._h, port, ctypes.byref(mm))
            return mm.value

        def sleep(self, sec):
            hakopy.usleep(int(sec * 1_000_000))

    state = {"hat": None, "handle": None, "samples": None}

    def on_initialize(_ctx):
        h = lib.spikehat_open(CONFIG_PATH.encode())
        if not h:
            return -1
        state["handle"] = h
        state["hat"] = HakoniwaHat(h)
        print("[INFO] spikehat opened", file=sys.stderr)
        return 0

    def on_reset(_ctx):
        return 0

    def on_manual_timing_control(_ctx):
        hat = state["hat"]
        if hat is None:
            return -1
        state["samples"] = do_scan(hat)
        lib.spikehat_close(ctypes.c_void_p(state["handle"]))
        return 0

    cb = {
        "on_initialize":            on_initialize,
        "on_simulation_step":       None,
        "on_manual_timing_control": on_manual_timing_control,
        "on_reset":                 on_reset,
    }

    ret = hakopy.asset_register(
        ASSET_NAME, PDU_DEF_PATH, cb, STEP_USEC,
        hakopy.HAKO_ASSET_MODEL_CONTROLLER)
    if not ret:
        print("[ERROR] hakopy.asset_register failed", file=sys.stderr)
        sys.exit(1)

    print(f"[INFO] '{ASSET_NAME}' 登録完了。hako-cmd start を待機中...", file=sys.stderr)
    hakopy.start()

    samples = state.get("samples") or []
    result = {"summary": make_summary(samples), "samples": samples}
    out = json.dumps(result, ensure_ascii=False, indent=2)
    if output_path:
        with open(output_path, "w") as f:
            f.write(out)
        print(f"→ {output_path} に保存", file=sys.stderr)
    else:
        print(out)


# ─── エントリーポイント ────────────────────────────────────────────────────

if __name__ == "__main__":
    output_path = sys.argv[1] if len(sys.argv) > 1 else None

    # hakopy がインポートできれば Hakoniwa モード
    try:
        import hakopy
        run_hakoniwa(output_path)
    except ImportError:
        run_real(output_path)
