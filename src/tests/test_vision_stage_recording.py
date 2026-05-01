import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

from config import CAM_FPS_RAW
from sbvs.camera.stereo_camera import StereoCamera
from sbvs.object.object_processor import ObjectProcessor
from sbvs.stereo.stereo_processor import StereoProcessor


WINDOW_NAME = "Vision Stage Recorder"
DEFAULT_OUTPUT_ROOT = Path("data/test/vision_stage_recordings")


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record stereo input and save each vision pipeline stage as its own video."
    )
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_OUTPUT_ROOT,
        help="Directory where timestamped recording folders are created.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=float(CAM_FPS_RAW),
        help="Output video FPS.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Optional auto-stop recording duration in seconds. 0 means manual stop.",
    )
    parser.add_argument(
        "--preview-width",
        type=int,
        default=1600,
        help="Preview window width in pixels.",
    )
    parser.add_argument(
        "--conf-threshold",
        type=float,
        default=0.35,
        help="YOLO confidence threshold for detection overlays.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show the live 2x2 preview window while recording.",
    )
    return parser.parse_args()


def ensure_color(frame: np.ndarray) -> np.ndarray:
    if frame.ndim == 2:
        return cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
    return frame


def normalize_u8(frame: np.ndarray) -> np.ndarray:
    if frame.dtype == np.uint8:
        return frame

    frame = np.nan_to_num(frame, nan=0.0, posinf=0.0, neginf=0.0)
    if frame.size == 0:
        return np.zeros((1, 1), dtype=np.uint8)

    min_val = float(frame.min())
    max_val = float(frame.max())
    if max_val - min_val < 1e-6:
        return np.zeros(frame.shape, dtype=np.uint8)

    scaled = (frame - min_val) * (255.0 / (max_val - min_val))
    return scaled.astype(np.uint8)


def colorize_disparity(disparity: np.ndarray) -> np.ndarray:
    disp_u8 = normalize_u8(disparity)
    return cv2.applyColorMap(disp_u8, cv2.COLORMAP_TURBO)


def build_raw_frame(left_bgr: np.ndarray, right_bgr: np.ndarray) -> np.ndarray:
    return cv2.hconcat([left_bgr, right_bgr])


def build_rectified_frame(rect_left: np.ndarray, rect_right: np.ndarray) -> np.ndarray:
    rect_left_bgr = ensure_color(normalize_u8(rect_left))
    rect_right_bgr = ensure_color(normalize_u8(rect_right))
    return cv2.hconcat([rect_left_bgr, rect_right_bgr])


def fit_to_canvas(frame: np.ndarray, target_width: int, target_height: int) -> np.ndarray:
    frame = ensure_color(frame)
    src_height, src_width = frame.shape[:2]
    scale = min(target_width / src_width, target_height / src_height)
    resized_width = max(1, int(src_width * scale))
    resized_height = max(1, int(src_height * scale))
    resized = cv2.resize(frame, (resized_width, resized_height), interpolation=cv2.INTER_LINEAR)

    canvas = np.zeros((target_height, target_width, 3), dtype=np.uint8)
    y_offset = (target_height - resized_height) // 2
    x_offset = (target_width - resized_width) // 2
    canvas[y_offset:y_offset + resized_height, x_offset:x_offset + resized_width] = resized
    return canvas


def build_preview(raw_frame: np.ndarray, rectified_frame: np.ndarray,
                  disparity_frame: np.ndarray, detection_frame: np.ndarray) -> np.ndarray:
    tiles = [raw_frame, rectified_frame, disparity_frame, detection_frame]
    target_height = min(tile.shape[0] for tile in tiles)
    target_width = max(
        1,
        max(int(tile.shape[1] * (target_height / tile.shape[0])) for tile in tiles),
    )

    fitted = [fit_to_canvas(tile, target_width, target_height) for tile in tiles]

    top_row = cv2.hconcat(fitted[:2])
    bottom_row = cv2.hconcat(fitted[2:])
    return cv2.vconcat([top_row, bottom_row])


class StageWriters:
    def __init__(self, output_dir: Path, fps: float):
        self.output_dir = output_dir
        self.fps = fps
        self.codec = cv2.VideoWriter_fourcc(*"mp4v")
        self._writers = {}

    def _open_writer(self, name: str, frame: np.ndarray):
        path = self.output_dir / f"{name}.mp4"
        height, width = frame.shape[:2]
        writer = cv2.VideoWriter(str(path), self.codec, self.fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"Failed to open video writer for {path}")
        self._writers[name] = writer

    def write(self, name: str, frame: np.ndarray):
        frame = ensure_color(frame)
        if name not in self._writers:
            self._open_writer(name, frame)
        self._writers[name].write(frame)

    def close(self):
        for writer in self._writers.values():
            writer.release()
        self._writers.clear()


def write_metadata(output_dir: Path, args, frame_count: int, started_at: float, ended_at: float):
    metadata = {
        "created_at": datetime.now().isoformat(),
        "frame_count": frame_count,
        "fps": args.fps,
        "duration_seconds": max(0.0, ended_at - started_at),
        "videos": {
            "raw": "raw.mp4",
            "rectified": "rectified.mp4",
            "disparity": "disparity.mp4",
            "detection": "detection.mp4",
        },
        "settings": {
            "conf_threshold": args.conf_threshold,
            "auto_stop_duration": args.duration,
        },
    }
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def start_recording_session(output_root: Path, fps: float):
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = output_root / f"recording_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=True)
    writers = StageWriters(output_dir, fps)
    started_at = time.monotonic()
    print(f"[REC] Recording started: {output_dir}")
    return writers, output_dir, started_at, 0


def main():
    args = parse_args()

    output_root = args.output_root
    output_root.mkdir(parents=True, exist_ok=True)

    proc_stereo = StereoProcessor()
    proc_obj = ObjectProcessor(
        crop_factor=0.2,
        confidence_history_len=25,
        min_hits=5,
        use_builtin_tracking=False,
    )
    camera = StereoCamera(sync_tolerance_ms=15.0, max_queue=2, vpi_convert=True)

    recording = False
    writers = None
    output_dir = None
    recording_started_at = None
    recorded_frames = 0

    if args.preview:
        cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WINDOW_NAME, args.preview_width, max(900, args.preview_width // 2))

    try:
        with camera:
            if args.preview:
                print("[REC] Preview ready. Press R to start/stop recording, Q or ESC to quit.")
            else:
                print("[REC] Headless mode ready. Recording starts immediately. Stop with Ctrl+C or --duration.")
                writers, output_dir, recording_started_at, recorded_frames = start_recording_session(output_root, args.fps)
                recording = True

            for frame_left_vpi, frame_right_vpi, _, _ in camera.frames():
                raw_left = frame_left_vpi.cpu()
                raw_right = frame_right_vpi.cpu()

                stereo_result = proc_stereo.run(frame_left_vpi, frame_right_vpi)
                rect_left = proc_stereo.vpi_rectL.cpu()
                rect_right = proc_stereo.vpi_rectR.cpu()
                disparity_frame = colorize_disparity(stereo_result.disparity)

                detection_frame, detections = proc_obj.process(
                    frame_left_vpi,
                    stereo_result,
                    render=True,
                    conf_threshold=args.conf_threshold,
                )
                proc_obj.draw_info(detection_frame, detections)

                raw_frame = build_raw_frame(raw_left, raw_right)
                rectified_frame = build_rectified_frame(rect_left, rect_right)
                detection_frame = ensure_color(detection_frame)

                if args.preview:
                    preview = build_preview(raw_frame, rectified_frame, disparity_frame, detection_frame)
                    status = "RECORDING" if recording else "PREVIEW"
                    cv2.putText(
                        preview,
                        f"{status} | r=start/stop q=quit",
                        (20, 40),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0) if recording else (255, 255, 255),
                        2,
                        cv2.LINE_AA,
                    )
                    cv2.imshow(WINDOW_NAME, preview)

                if recording:
                    writers.write("raw", raw_frame)
                    writers.write("rectified", rectified_frame)
                    writers.write("disparity", disparity_frame)
                    writers.write("detection", detection_frame)
                    recorded_frames += 1

                    if args.duration > 0 and (time.monotonic() - recording_started_at) >= args.duration:
                        recording = False
                        writers.close()
                        write_metadata(output_dir, args, recorded_frames, recording_started_at, time.monotonic())
                        print(f"[REC] Auto-stopped recording after {args.duration:.1f}s: {output_dir}")
                        writers = None
                        output_dir = None
                        recording_started_at = None
                        recorded_frames = 0

                key = cv2.waitKey(1) & 0xFF if args.preview else -1
                if key in (ord("q"), 27):
                    break

                if key == ord("r"):
                    if not recording:
                        writers, output_dir, recording_started_at, recorded_frames = start_recording_session(output_root, args.fps)
                        recording = True
                    else:
                        recording = False
                        writers.close()
                        write_metadata(output_dir, args, recorded_frames, recording_started_at, time.monotonic())
                        print(f"[REC] Recording stopped: {output_dir}")
                        writers = None
                        output_dir = None
                        recording_started_at = None
                        recorded_frames = 0
    finally:
        if writers is not None:
            writers.close()
            if output_dir is not None and recording_started_at is not None:
                write_metadata(output_dir, args, recorded_frames, recording_started_at, time.monotonic())
        cv2.destroyAllWindows()
        print("[REC] Camera stats:", camera.get_stats())


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n[REC] Recording interrupted by user.")
