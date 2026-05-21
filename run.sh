#!/bin/bash
# Build HAT ファームウェアをロードしてからスキャナーを起動する
python3 -c "from buildhat import Motor; Motor('A')" 2>/dev/null
python3 /home/kuboaki/projects/sonar_radar/sonar_radar.py "$@"
