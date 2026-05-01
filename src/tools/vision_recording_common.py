import json
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np


DEFAULT_RAW_OUTPUT_ROOT = Path("data/test/raw_stereo_recordings")
DEFAULT_RENDER_OUTPUT_ROOT = Path("data/test/vision_stage_recordings")


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
    return cv2.hconcat([ensure_color(left_bgr), ensure_color(right_bgr)])


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


def build_preview(*tiles: np.ndarray) -> np.ndarray:
    target_height = min(tile.shape[0] for tile in tiles)
    target_width = max(1, max(int(tile.shape[1] * (target_height / tile.shape[0])) for tile in tiles))
    fitted = [fit_to_canvas(tile, target_width, target_height) for tile in tiles]
    top_row = cv2.hconcat(fitted[:2])
    bottom_row = cv2.hconcat(fitted[2:])
    return cv2.vconcat([top_row, bottom_row])


class VideoWriters:
    def __init__(self, output_dir: Path, fps: float, codec: str = "mp4v"):
        self.output_dir = output_dir
        self.fps = fps
        self.codec = cv2.VideoWriter_fourcc(*codec)
        self._writers: dict[str, cv2.VideoWriter] = {}

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


def make_timestamped_dir(root: Path, prefix: str) -> Path:
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = root / f"{prefix}_{stamp}"
    output_dir.mkdir(parents=True, exist_ok=True)
    return output_dir


def write_metadata(output_dir: Path, payload: dict):
    metadata = dict(payload)
    metadata.setdefault("created_at", datetime.now().isoformat())
    (output_dir / "metadata.json").write_text(json.dumps(metadata, indent=2), encoding="utf-8")


def monotonic_duration(started_at: float | None, ended_at: float | None = None) -> float:
    if started_at is None:
        return 0.0
    end = time.monotonic() if ended_at is None else ended_at
    return max(0.0, end - started_at)

