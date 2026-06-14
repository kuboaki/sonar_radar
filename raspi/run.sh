#!/bin/bash
# Build HAT ファームウェアをロードしてからスキャナーを起動する
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
python3 -c "from buildhat import Motor; Motor('A')" 2>/dev/null
python3 "$SCRIPT_DIR/sonar_radar.py" "$@"
