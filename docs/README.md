# SnoBot

SnoBot is an autonomous snow-removal robot project that combines embedded motor control, stereo vision, teleoperation, odometry, and coverage planning. The codebase is split between Python services running on the host computer and Arduino/ESP32 firmware for robot control and the handheld remote.

## What is in this repo

- `src/main_auto.py`: autonomous runtime that connects the control loop to the stereo vision pipeline and obstacle handling
- `src/main_remote.py`: remote-control server for the ESP32 handheld controller over Bluetooth RFCOMM
- `src/tools/odometry_web_view.py`: local web viewer for odometry, perimeter recording, and path preview
- `src/arduino` and `src/esp32`: firmware and embedded support code
- `src/sbcp`: robot control protocol, transport, commands, and state machine
- `src/sban`: autonomy modules such as odometry, perimeter recording, obstacle behavior, and coverage planning
- `src/sbvs`: stereo vision, calibration, camera handling, and object detection
- `docs/`: project notes and protocol/design documentation
- `data/`: calibration files, protocol data, model assets, and saved perimeter/test artifacts

## Quick start

This repository does not currently include a pinned `requirements.txt` or `pyproject.toml`, so environment setup is manual. From the code, the Python stack expects local access to project modules under `src/` and uses components such as OpenCV, NVIDIA VPI, and Ultralytics YOLO for the vision path.

Typical entry points:

```bash
python src/main_remote.py --dummy
python src/main_auto.py --dummy --no-vision
python src/tools/odometry_web_view.py --port COM12
```

Helper launchers for the odometry viewer are also included:

```bash
./start_odometry_web.sh
start_odometry_web.bat
```

## Notes

- Default serial settings in the Python entry points target `115200` baud.
- Saved perimeters are written under `data/sban/perimeters`.
- Vision calibration and model assets live under `data/sbvs`.
- Additional project background is in [docs/README.md](docs/README.md) and the write-up under `ECE_499_Project_Writeup/`.
