import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import vpi

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from config import CAM_FPS_RAW
from sbvs.object.object_processor import ObjectProcessor
from sbvs.stereo.stereo_processor import StereoProcessor
from tools.vision_recording_common import (
    DEFAULT_RENDER_OUTPUT_ROOT,
    VideoWriters,
    build_preview,
    build_raw_frame,
    build_rectified_frame,
    colorize_disparity,
    ensure_color,
    make_timestamped_dir,
    monotonic_duration,
    write_metadata,
)


WINDOW_NAME = "Vision Recording Renderer"


def parse_args():
    parser = argparse.ArgumentParser(
        description="Render rectified, disparity, and detection videos from a saved raw stereo recording."
    )
    parser.add_argument(
        "input_dir",
        type=Path,
        help="A raw recording directory containing left.mp4, right.mp4, and metadata.json.",
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_RENDER_OUTPUT_ROOT,
        help="Directory where timestamped rendered folders are created.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=0.0,
        help="Override output FPS. Defaults to the raw recording metadata FPS.",
    )
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=0.35,
        help="YOLO confidence threshold for rendered detections.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show a live 2x2 preview while rendering.",
    )
    return parser.parse_args()


def load_metadata(input_dir: Path) -> dict:
    metadata_path = input_dir / "metadata.json"
    if not metadata_path.exists():
        return {}
    return json.loads(metadata_path.read_text(encoding="utf-8"))


def open_video(path: Path) -> cv2.VideoCapture:
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise RuntimeError(f"Failed to open video: {path}")
    return cap


def main():
    args = parse_args()
    metadata = load_metadata(args.input_dir)

    left_path = args.input_dir / "left.mp4"
    right_path = args.input_dir / "right.mp4"
    if not left_path.exists() or not right_path.exists():
        raise FileNotFoundError("Expected left.mp4 and right.mp4 in the input recording directory.")

    source_fps = metadata.get("effective_fps", metadata.get("fps", CAM_FPS_RAW))
    fps = args.fps or float(source_fps)
    args.output_root.mkdir(parents=True, exist_ok=True)
    output_dir = make_timestamped_dir(args.output_root, "recording")
    writers = VideoWriters(output_dir, fps)
    left_cap = open_video(left_path)
    right_cap = open_video(right_path)
    stereo = StereoProcessor()
    objects = ObjectProcessor(
        crop_factor=0.2,
        confidence_history_len=25,
        min_hits=5,
        use_builtin_tracking=False,
    )
    started_at = time.monotonic()
    frame_count = 0

    if args.preview:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, 1600, 900)

    print(f"[RENDER] Reading raw recording from {args.input_dir}")
    print(f"[RENDER] Writing stage videos to {output_dir}")

    try:
        while True:
            ok_left, left_frame = left_cap.read()
            ok_right, right_frame = right_cap.read()
            if not ok_left or not ok_right:
                break

            with vpi.Backend.CUDA:
                left_vpi = vpi.asimage(left_frame)
                right_vpi = vpi.asimage(right_frame)

            stereo_result = stereo.run(left_vpi, right_vpi)
            rect_left = stereo.vpi_rectL.cpu()
            rect_right = stereo.vpi_rectR.cpu()
            disparity_frame = colorize_disparity(stereo_result.disparity)

            detection_frame, detections = objects.process(
                left_vpi,
                stereo_result,
                render=True,
                conf_threshold=args.conf_threshold,
            )
            objects.draw_info(detection_frame, detections)

            raw_frame = build_raw_frame(left_frame, right_frame)
            rectified_frame = build_rectified_frame(rect_left, rect_right)
            detection_frame = ensure_color(detection_frame)

            writers.write("raw", raw_frame)
            writers.write("rectified", rectified_frame)
            writers.write("disparity", disparity_frame)
            writers.write("detection", detection_frame)
            frame_count += 1

            if args.preview:
                preview = build_preview(raw_frame, rectified_frame, disparity_frame, detection_frame)
                cv2.putText(
                    preview,
                    f"RENDER {frame_count} frames",
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
    finally:
        left_cap.release()
        right_cap.release()
        writers.close()
        write_metadata(
            output_dir,
            {
                "recording_type": "rendered_vision_stages",
                "source_recording": str(args.input_dir),
                "frame_count": frame_count,
                "fps": fps,
                "duration_seconds": monotonic_duration(started_at),
                "source_input_fps": metadata.get("fps"),
                "source_effective_fps": metadata.get("effective_fps"),
                "videos": {
                    "raw": "raw.mp4",
                    "rectified": "rectified.mp4",
                    "disparity": "disparity.mp4",
                    "detection": "detection.mp4",
                },
                "settings": {
                    "conf_threshold": args.conf_threshold,
                },
            },
        )
        cv2.destroyAllWindows()
        print(f"[RENDER] Rendered {frame_count} frames to {output_dir}")


if __name__ == "__main__":
    main()
