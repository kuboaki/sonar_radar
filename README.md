# sonar_radar

2D radar scanner using LEGO SPIKE Prime + Raspberry Pi Build HAT via [libspikehat](https://github.com/kuboaki/libspikehat).

[日本語版 README](README_ja.md)

## Hardware Configuration

| Port | Device | Role |
|------|--------|------|
| A (0) | SPIKE Prime L Angular Motor | Arm rotation |
| C (2) | SPIKE Prime Color Sensor | End-stop marker detection |
| D (3) | SPIKE Prime Distance Sensor | Obstacle measurement |

### Rotation Markers

- **Red marker** — left end (negative direction)
- **Blue marker** — right end (positive direction)

Place markers at ±35° or ±65° positions to define the scan range.

## Scan Specification

| Parameter | Value |
|-----------|-------|
| Scan range | ±35° or ±65° (selectable) |
| Step angle | 5° |
| Valid distance | 150–500 mm |
| Origin (0°) | Front center |

## Requirements

- Raspberry Pi 4 + Build HAT
- **Raspberry Pi OS Bookworm (64-bit)**
- [libspikehat](https://github.com/kuboaki/libspikehat) built and installed
- `python3-build-hat` installed (`sudo apt install python3-build-hat`)

## Usage

```bash
bash run.sh            # Standard scan (±65°)
bash run.sh --range 35 # Narrow scan (±35°)
```

`run.sh` loads the Build HAT firmware before scanning. Use it instead of calling `sonar_radar.py` directly.

## Output

JSON array to stdout, log to stderr:

```json
[
  {"angle": -65, "distance_mm": 312},
  {"angle": -60, "distance_mm": 298},
  {"angle": -55, "distance_mm": null},
  ...
]
```

`distance_mm` is `null` when no object is detected within the valid range.

## Calibration

On startup, the arm rotates counter-clockwise until the color sensor detects the red marker (left end), then moves to 0° (front). No manual pre-positioning is required.

## Project Structure

```
sonar_radar/
├── sonar_radar.py   Main scanner script
└── run.sh           Startup script (loads firmware + runs scanner)
```

## License

MIT License
