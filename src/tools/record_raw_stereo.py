import argparse
import sys
import time
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CAM_FPS_RAW, CAM_HEIGHT_RAW, CAM_ID_LEFT, CAM_ID_RIGHT, CAM_WIDTH_RAW
from sbvs.camera.stereo_camera import StereoCamera
from tools.vision_recording_common import (
    DEFAULT_RAW_OUTPUT_ROOT,
    VideoWriters,
    build_raw_frame,
    make_timestamped_dir,
    monotonic_duration,
    write_metadata,
)


WINDOW_NAME = "Raw Stereo Recorder"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record synchronized raw video from the left and right cameras."
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_RAW_OUTPUT_ROOT,
        help="Directory where timestamped recording folders are created.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=float(CAM_FPS_RAW),
        help="Output video FPS for saved files.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional auto-stop duration in seconds. 0 means run until Ctrl+C or q.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a live side-by-side preview window while recording.",
    )
    parser.add_argument(
        "--sync-tolerance-ms",
        type=float,
        default=15.0,
        help="Maximum timestamp delta allowed when pairing left/right frames.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    args.output_root.mkdir(parents=True, exist_ok=True)

    output_dir = make_timestamped_dir(args.output_root, "raw_recording")
    writers = VideoWriters(output_dir, args.fps)
    camera = StereoCamera(sync_tolerance_ms=args.sync_tolerance_ms, max_queue=2, vpi_convert=False)
    started_at = time.monotonic()
    frame_count = 0
    ts_left = None
    ts_right = None

    if args.preview:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1600, 900)

    print(f"[RAW] Recording to {output_dir}")
    print("[RAW] Press q in the preview window or Ctrl+C to stop.")

    try:
        with camera:
            for left_frame, right_frame, ts_left, ts_right in camera.frames():
                writers.write("left", left_frame)
                writers.write("right", right_frame)
                frame_count += 1

                if args.preview:
                    preview = build_raw_frame(left_frame, right_frame)
                    cv2.putText(
                        preview,
                        f"REC {frame_count} frames",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow(WINDOW_NAME, preview)
                    key = cv2.waitKey(1) & 0xFF
                    if key in (ord("q"), 27):
                        break

                if args.duration > 0 and monotonic_duration(started_at) >= args.duration:
                    break
    finally:
        writers.close()
        duration_seconds = monotonic_duration(started_at)
        effective_fps = (frame_count / duration_seconds) if duration_seconds > 0 else 0.0
        camera_stats = camera.get_stats()
        write_metadata(
            output_dir,
            {
                "recording_type": "raw_stereo",
                "frame_count": frame_count,
                "fps": args.fps,
                "effective_fps": effective_fps,
                "duration_seconds": duration_seconds,
                "videos": {
                    "left": "left.mp4",
                    "right": "right.mp4",
                },
                "camera": {
                    "left_id": CAM_ID_LEFT,
                    "right_id": CAM_ID_RIGHT,
                    "width": CAM_WIDTH_RAW,
                    "height": CAM_HEIGHT_RAW,
                    "sync_tolerance_ms": args.sync_tolerance_ms,
                },
                "timing": {
                    "last_pair_timestamps": {
                        "left_monotonic_s": ts_left if frame_count else None,
                        "right_monotonic_s": ts_right if frame_count else None,
                    },
                },
                "stats": camera_stats,
            },
        )
        cv2.destroyAllWindows()
        print(f"[RAW] Saved {frame_count} synchronized frame pairs to {output_dir}")
        print(f"[RAW] Effective saved FPS: {effective_fps:.2f}")
        print(f"[RAW] Camera stats: {camera_stats}")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[RAW] Recording interrupted by user.")
