import cv2
import threading
import time

from collections import deque
from typing import Any, Deque, Optional, Tuple

from sbvs.camera.pipeline import gstreamer_pipeline
from config import CAM_MAX_FAIL, THREAD_BARRIER_TIMEOUT, THREAD_ERROR_SLEEP, THREAD_JOIN_TIMEOUT, THREAD_READ_FAIL_SLEEP

class CameraThread(threading.Thread):
    """
    Thread that continuously reads frames from a camera and stores the most recent ones.\n
    Each entry in the queue is: (timestamp_sec, frame).
    """
    
    def __init__(
        self, 
        sensor_id: int, 
        out_format: str = "BGR", 
        max_queue: int = 4, 
        barrier: threading.Barrier | None = None
    ):
        super().__init__(daemon=True, name=f"CameraThread_{sensor_id}")
        
        self.sensor_id = sensor_id
        self.out_format = out_format
        self.pipeline = gstreamer_pipeline(sensor_id, out_format)
        self.capture = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        if not self.capture.isOpened():
            raise RuntimeError(f"[CAMTHREAD] Failed to open camera {sensor_id} with pipeline: \n{self.pipeline}")
        self.queue: Deque[Tuple[float, Any]] = deque(maxlen=max_queue)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self.barrier = barrier

        self.frames_read = 0
        self.frames_dropped = 0
        
    def run(self):
        if self.barrier is not None:
            try:
                self.barrier.wait(timeout=THREAD_BARRIER_TIMEOUT)
            except threading.BrokenBarrierError:
                print(f"[CAMTHREAD] Barrier broken for camera {self.sensor_id}, aborting...")
                return
            except Exception as e:
                print(f"[CAMTHREAD] Barrier wait failed for camera {self.sensor_id}: {e}")
                return

        consecutive_failures = 0

        while not self._stop_event.is_set():
            try:
                self.capture.grab()
                ret, frame = self.capture.retrieve()

                if not ret or frame is None:
                    consecutive_failures += 1
                    if consecutive_failures >= CAM_MAX_FAIL:
                        print(f"[CAMTHREAD] Camera {self.sensor_id} failed {CAM_MAX_FAIL} times, stopping...")
                        break
                    time.sleep(THREAD_READ_FAIL_SLEEP)
                    continue

                ts = time.monotonic()
                consecutive_failures = 0
                self.frames_read += 1

                with self._lock:
                    if len(self.queue) == self.queue.maxlen:
                        self.frames_dropped += 1
                    self.queue.append((ts, frame))

            except cv2.error as e:
                print(f"[CAMTHREAD] OpenCV error on camera {self.sensor_id}: {e}")
                time.sleep(THREAD_ERROR_SLEEP)

            except Exception:
                # Programming error â€” crash loudly
                print(f"[CAMTHREAD] Unexpected error on camera {self.sensor_id}")
                raise

    def stop(self):
        self._stop_event.set()

    def release(self):
        self.stop()

        if self.is_alive():
            self.join(timeout=THREAD_JOIN_TIMEOUT)
            if self.is_alive():
                print(f"[CAMTHREAD] Warning: thread {self.name} did not stop cleanly.")

        with self._lock:
            self.queue.clear()

        try:
            if self.capture is not None:
                self.capture.release()
                self.capture = None
        except Exception as e:
            print(f"[CAMTHREAD] Error releasing camera: {e}")
            
    def peek(self) -> tuple[float, Any] | None:
        """Return the oldest frame in the queue without removing it."""
        with self._lock:
            return self.queue[0] if self.queue else None

    def pop(self) -> tuple[float, Any] | None:
        """Remove and return the oldest frame from the queue."""
        with self._lock:
            return self.queue.popleft() if self.queue else None

    def get_stats(self) -> dict:
        """Return frames read, dropped, and queue occupancy."""
        with self._lock:
            queue_len = len(self.queue)
        return {
            "frames_read": self.frames_read,
            "frames_dropped": self.frames_dropped,
            "queue_len": queue_len
        }