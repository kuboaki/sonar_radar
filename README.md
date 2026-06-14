# sonar_radar

2D radar scanner using LEGO SPIKE Prime + Raspberry Pi Build HAT via [libspikehat](https://github.com/kuboaki/libspikehat).

This project is developed as a **digital twin**: the same application code
(`raspi/sonar_radar.py`) runs either on the real hardware or in a MuJoCo
simulation (`sim/sonar_radar_sim.py` + `libspikehat_sim`), so new behavior
can be developed and tuned in simulation before running it on the real robot.

![sonar_radar overview](docs/sonar_radar_overview.jpg)

[日本語版 README](README_ja.md)

## Hardware Configuration

| Port | Device | Role |
|------|--------|------|
| A (0) | SPIKE Prime L Angular Motor | Dome rotation (1:3 reduction gear, direction-reversed) |
| B (1) | SPIKE Prime Force Sensor | Scan stop switch |
| C (2) | SPIKE Prime Color Sensor | End-stop marker detection (red = left end, blue = right end) |
| D (3) | SPIKE Prime Distance Sensor | Obstacle measurement |

### Rotation Markers

- **Red marker** — left end (negative direction)
- **Blue marker** — right end (positive direction)

The dome scans back and forth, reversing direction whenever it detects either marker,
until the force sensor (stop switch) is pressed.

## Scan Specification

| Parameter | Value |
|-----------|-------|
| Sampling interval | 50 ms |
| Valid distance | 50–300 mm |
| Origin (0°) | Front center (calibrated at startup) |

## Running on the Real Hardware

- Raspberry Pi 4 + Build HAT
- **Raspberry Pi OS Bookworm (64-bit)**
- [libspikehat](https://github.com/kuboaki/libspikehat) built and installed
- `python3-build-hat` installed (`sudo apt install python3-build-hat`)

```bash
cd raspi
bash run.sh
```

`run.sh` loads the Build HAT firmware before running `sonar_radar.py`.
Avoid calling `sonar_radar.py` directly.

## Running in Simulation (MuJoCo)

- macOS / Linux with `mujoco` and `libspikehat_sim` installed (see `sim/libspikehat_sim/`)
- For viewer mode on macOS, use `mjpython` (required by MuJoCo's passive viewer)

```bash
cd sim
python3 sonar_radar_sim.py            # batch run (prints JSON result to stdout)
mjpython sonar_radar_sim.py --viewer  # with 3D viewer, real-time
```

`sonar_radar_sim.py` injects a simulated `spikehat` module backed by MuJoCo
and then runs `raspi/sonar_radar.py` unmodified — any change made to
`sonar_radar.py` is immediately reflected in the simulation. The simulation
always runs at real-time speed (`--speed 1.0`, fixed) so its behavior can be
compared directly against the real hardware.

The Control tab in the viewer lets you move the obstacle wall (`wall_x_ctrl`,
`wall_y_ctrl`) and press the stop switch (`press_ctrl`) interactively. See
[mujoco_model/studio_to_mujoco.md](mujoco_model/studio_to_mujoco.md) for how
the MuJoCo model itself is built from the Bricklink Studio design.

![シミュレーション実行中](docs/sonar_radar_sim_snap.png)

*シミュレーション実行中*

## Output

JSON array to stdout, log to stderr:

```json
[
  {"angle": 12, "dome_angle": -4.0, "distance_mm": 136},
  {"angle": 15, "dome_angle": -5.0, "distance_mm": 135},
  {"angle": 21, "dome_angle": -7.0, "distance_mm": null},
  ...
]
```

- `angle` — motor encoder angle (degrees, relative to calibrated zero)
- `dome_angle` — dome angle (degrees, `angle / -3`)
- `distance_mm` — `null` when no object is detected within the valid range

## Visualization

`raspi/sonar_plot.py` reads the JSON output and plots `dome_angle` vs
`distance_mm` as a fan-shaped polar plot and a line plot, overlaying each
back-and-forth scan pass in a different color:

```bash
python3 raspi/sonar_radar.py > scan.json
python3 raspi/sonar_plot.py scan.json -o scan_result.png --title "scan result"
```

| Real hardware (`docs/scan_real.json`) | Simulation (`docs/scan_sim.json`) |
|---|---|
| ![real scan example](docs/scan_real_example.png) | ![sim scan example](docs/scan_sim_example.png) |

The real ultrasonic distance sensor has a wide beam, so it detects the wall
across a wide angular range (roughly -45° to +35° here). The simulated
distance sensor uses a single raycast, so it only detects the wall in a
narrow angular range directly in front (roughly -21° to +1° here). This FOV
discrepancy between the real sensor and the simulation is a known, currently
unaddressed difference (see [mujoco_model/studio_to_mujoco.md](mujoco_model/studio_to_mujoco.md)).

## Calibration

On startup, the motor moves to its mechanical zero position, then rotates by
`SENSOR_HOME_OFFSET` to compensate for the gear meshing offset, so that the
dome faces front (0°). No manual pre-positioning is required.

## Project Structure

```
sonar_radar/
├── raspi/                  Real-hardware app (run on Raspberry Pi)
│   ├── sonar_radar.py    Main scanner script (shared with simulation)
│   └── run.sh              Startup script (loads firmware + runs scanner)
├── sim/                     MuJoCo simulation
│   ├── sonar_radar_sim.py  Entry point that runs sonar_radar.py via libspikehat_sim
│   └── libspikehat_sim/    MuJoCo-based simulation library (libspikehat API)
├── mujoco_model/            MuJoCo model (XML, meshes, Blender export scripts)
│   └── studio_to_mujoco.md  Bricklink Studio → MuJoCo model build guide
├── studio_model/            Bricklink Studio model files
└── docs/                    Documentation images
```

## License

MIT License
