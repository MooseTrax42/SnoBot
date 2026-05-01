import numpy as np
import threading
import time
import vpi

from collections import deque
from typing import Any, Callable, Deque, Iterator, Optional, Tuple

from sbvs.camera.camera_thread import CameraThread
from config import CAM_ID_LEFT, CAM_ID_RIGHT, STEREOCAM_GENERATOR_SLEEP, STEREOCAM_STOP_GRACEFUL, STEREOCAM_CALIBRATION_SAMPLES, STEREOCAM_CALIBRATION_TIMEOUT, STEREOCAM_CALIBRATION_SLEEP, STEREOCAM_DELIVERY_SLEEP, STEREOCAM_GENERATOR_QUEUE_SIZE, STEREOCAM_JOIN_TIMEOUT, STEREOCAM_PAIR_QUEUE_SIZE, STEREOCAM_STARTUP_DELAY, STEREOCAM_OFFSET_TOLERANCE

class StereoCamera:
    """
    Manages two CameraThread instances, synchronizes frames by timestamp.
    
    Key params:
        - sync_tolerance_ms: maximum valid timestamp difference between lens inputs.
        - max_queue: per-camera queue size.
        - out_format: passed to GStreamer (BGR, GRAY8, etc.) 
    """
    
    def __init__(self, camL: int = CAM_ID_LEFT, camR: int = CAM_ID_RIGHT, out_format: str = "BGR", sync_tolerance_ms: float = 30.0, max_queue: int = 8, vpi_convert: bool = False):
        
        # Store initialization parameters for potential restarts
        self.camL = camL
        self.camR = camR
        self.out_format = out_format
        self.max_queue = max_queue
        
        self._init_camera_threads()
        
        self.sync_tolerance = sync_tolerance_ms / 1000 # Seconds.
        self._pair_queue: Deque[Tuple[float, Any, float, Any]] = deque(maxlen=max_queue)
        
        self._stop_event = threading.Event()
        self._deliver_thread: Optional[threading.Thread] = None
        self._callback: Optional[Callable[[Any, Any, float, float], None]] = None
        
        self.vpi_convert = vpi_convert
        self._sync_lock = threading.Lock()
        
        self.pairs_delivered = 0
        self.frames_dropped_L = 0
        self.frames_dropped_R = 0
        self.hardware_offset = 0.0
        self.pipeline_retries = 0
        self._started = False
        self._startup_done = threading.Event()

    def _init_camera_threads(self):
        """Initialize or reinitialize camera threads."""
        sync_barrier = threading.Barrier(2)
        
        self.left = CameraThread(self.camL, out_format=self.out_format, max_queue=self.max_queue, barrier=sync_barrier)
        self.right = CameraThread(self.camR, out_format=self.out_format, max_queue=self.max_queue, barrier=sync_barrier)
        
    def _calibrate_offset(self, samples=STEREOCAM_CALIBRATION_SAMPLES):
        """
        Measure offset, variability, and frame timing consistency.
        """
        
        offsets = []
        left_intervals = []
        right_intervals = []
        last_tsL, last_tsR = None, None
        
        timeout = time.time() + STEREOCAM_CALIBRATION_TIMEOUT
        
        while len(offsets) < samples and time.time() < timeout:
            tsfL = self.left.peek()
            tsfR = self.right.peek()

            if tsfL is not None and tsfR is not None:
                tsL, _ = tsfL
                tsR, _ = tsfR
                offsets.append(tsR - tsL)
                
                # Track frame intervals
                if last_tsL is not None:
                    left_intervals.append(tsL - last_tsL)
                if last_tsR is not None:
                    right_intervals.append(tsR - last_tsR)
                    
                last_tsL, last_tsR = tsL, tsR
                
            time.sleep(STEREOCAM_CALIBRATION_SLEEP)
        
        if offsets:
            median_offset = float(np.median(offsets))
            std_offset = float(np.std(offsets))
            left_mean = float(np.mean(left_intervals)) if left_intervals else 0.0
            left_std = float(np.std(left_intervals)) if left_intervals else 0.0
            right_mean = float(np.mean(right_intervals)) if right_intervals else 0.0
            right_std = float(np.std(right_intervals)) if right_intervals else 0.0

            print(f"[STEREO] Offset std dev: {std_offset*1000:.2f}ms")
            print(f"[STEREO] Left frame interval: {left_mean*1000:.2f}ms ± {left_std*1000:.2f}ms")
            print(f"[STEREO] Right frame interval: {right_mean*1000:.2f}ms ± {right_std*1000:.2f}ms")
    
                
            return {
                "median_offset": median_offset,
                "offset_std": std_offset,
                "left_interval_mean": left_mean,
                "left_interval_std": left_std,
                "right_interval_mean": right_mean,
                "right_interval_std": right_std
            }
        
        return {
            "median_offset": 0.0,
            "offset_std": 0.0,
            "left_interval_mean": 0.0,
            "left_interval_std": 0.0,
            "right_interval_mean": 0.0,
            "right_interval_std": 0.0
        }
        
    def start_threads(self):
        """Start camera threads (daemon threads)."""
        self.left.start()
        self.right.start()
        
    def stop_threads(self):
        self.left.release()
        self.right.release()
        
    def peek_pair(self) -> Optional[Tuple[float, Any, float, Any]]:
        """Return the oldest synchronized pair without removing it."""
        with self._sync_lock:
            return self._pair_queue[0] if self._pair_queue else None

    def pop_pair(self) -> Optional[Tuple[float, Any, float, Any]]:
        """Remove and return the oldest synchronized pair."""
        with self._sync_lock:
            return self._pair_queue.popleft() if self._pair_queue else None
            
    def _attempt_sync(self):
        """
        Try to find matching pairs from the two camera queues.
        """
        while True:
            tsfL = self.left.peek()
            tsfR = self.right.peek()

            if tsfL is None or tsfR is None:
                break

            tsL, frameL = tsfL
            tsR, frameR = tsfR
            dt = (tsL + self.hardware_offset) - tsR

            if abs(dt) <= self.sync_tolerance:
                # Accept as a synced pair
                self.left.pop()
                self.right.pop()
                with self._sync_lock:
                    self._pair_queue.append((tsL, frameL, tsR, frameR))
                    self.pairs_delivered += 1
            else:
                # Drop frame from the older camera
                if dt < 0:
                    self.left.pop()
                    self.frames_dropped_L += 1
                else:
                    self.right.pop()
                    self.frames_dropped_R += 1
    
    def _delivery_loop(self):
        """
        Background loop that periodically tries to sync frames.
        """
        while not self._stop_event.is_set():
            # Attempt to gather new synced pairs from camera queues.
            self._attempt_sync()
            
            # Deliver one pair per iteration.
            pair_to_deliver = self.pop_pair()
            
            if pair_to_deliver is not None:
                tsL, frameL, tsR, frameR = pair_to_deliver
                deliver_left, deliver_right = frameL, frameR
                
                if self.vpi_convert:
                    try:
                        with vpi.Backend.CUDA:
                            vpiL = vpi.asimage(frameL)
                            vpiR = vpi.asimage(frameR)
                            deliver_left, deliver_right = vpiL, vpiR
                    except Exception as e:
                        print(f"[STEREO] VPI conversion failed: {e}")
                        
                # Deliver the frame.
                if self._callback:
                    try:
                        self._callback(deliver_left, deliver_right, tsL, tsR)
                    except Exception as e:
                        print(f"[STEREO] Callback raised: {e}")
                else:
                    if not hasattr(self, "_frames_for_generator"):
                        self._frames_for_generator = deque(maxlen=STEREOCAM_GENERATOR_QUEUE_SIZE)
                    self._frames_for_generator.append((deliver_left, deliver_right, tsL, tsR))
            else:
                time.sleep(STEREOCAM_DELIVERY_SLEEP)
            
    # Public methods.
    def start(self, callback: Optional[Callable[[Any, Any, float, float], None]]=None):
        """
        Start camera threads and the delivery thread.
        If callback is provided, it is called for eached synchronized pair.
        """
        
        if self._started:
            return
        
        self._startup_done.clear()
        
        while True:
            self._stop_event.clear()
            self.start_threads() # Starts camera capture threads
            
            # Wait for cameras to stabilize and fill queues
            time.sleep(STEREOCAM_STARTUP_DELAY)
            
            # Calibrate the hardware offset
            self._offset_info = self._calibrate_offset()
            self.hardware_offset = self._offset_info["median_offset"]
            
            if abs(self.hardware_offset) > STEREOCAM_OFFSET_TOLERANCE:
                print(f"[STEREO] Calibrated hardware offset: {self.hardware_offset*1000:.2f}ms. Exceeds tolerance of +/-{STEREOCAM_OFFSET_TOLERANCE*1000:.2f}ms. RESTARTING CAMERA...")
                
                # Stop threads and clean up before retrying the loop
                self.stop_threads()
                
                # Reinitialize camera threads for next attempt
                self._init_camera_threads()
                
                # Reset latency tracking variables
                self.pairs_delivered = 0
                self.frames_dropped_L = 0
                self.frames_dropped_R = 0
                
                # Increment retry counter
                self.pipeline_retries += 1
                
                continue # Restart the while True loop
            else:
                print(f"[STEREO] Calibrated hardware offset: {self.hardware_offset*1000:.2f}ms. Within tolerance.")
                break # Exit the while True loop and proceed with the delivery thread
                
        self._deliver_thread = threading.Thread(target=self._delivery_loop, name="StereoDelivery", daemon=True)
        self._deliver_thread.start()
        self._started = True
        self._startup_done.set()
    
    def stop(self):
        """Stop everything and join threads."""
        self._stop_event.set()
        
        # Give threads time to see the stop event.
        time.sleep(STEREOCAM_STOP_GRACEFUL)
        
        # Stop delivery thread first.
        if self._deliver_thread and self._deliver_thread.is_alive():
            self._deliver_thread.join(timeout=STEREOCAM_JOIN_TIMEOUT)
            if self._deliver_thread.is_alive():
                print("[STEREO] Warning: delivery thread did not stop cleanly.")
        
        # Stop camera threads.
        self.stop_threads()
        
        # Clear queues.
        with self._sync_lock:
            self._pair_queue.clear()
        if hasattr(self, "_frames_for_generator"):
            self._frames_for_generator.clear()
        self._started = False
        
    def frames(self) -> Iterator[Tuple[Any, Any, float, float]]:
        """Generator that gives synchronized pairs."""
        
        if not self._started:
            self.start(callback=None)
            
        # Consume from internal deque that the delivery loop fills.
        if not hasattr(self, "_frames_for_generator"):
            self._frames_for_generator = deque(maxlen=STEREOCAM_GENERATOR_QUEUE_SIZE)
           
        while not self._stop_event.is_set():
            if self._frames_for_generator:
                # Consume the OLDEST frame available
                fL, fR, tL, tR = self._frames_for_generator.popleft()
                yield fL, fR, tL, tR
            else:
                # Wait briefly if the queue is empty
                time.sleep(STEREOCAM_GENERATOR_SLEEP)
                
    # Handy context manager.
    def __enter__(self):
        self.start(callback=None)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        self.stop()
        
    def get_stats(self) -> dict:
        """Returns diagnostic information on throughput and drops."""
        return {
            "delivered_pairs": self.pairs_delivered,
            "L_read": self.left.frames_read,
            "R_read": self.right.frames_read,
            "L_dropped": self.frames_dropped_L,
            "R_dropped": self.frames_dropped_R,
            "sync_success_rate": self.pairs_delivered / (self.left.frames_read or 1),
            "pipeline_retries": self.pipeline_retries
        }
        
    def get_startup_info(self) -> dict:
        return {
            "offset_std_ms": self._offset_info["offset_std"] * 1000,
            "left_interval_mean_ms": self._offset_info["left_interval_mean"] * 1000,
            "left_interval_std_ms": self._offset_info["left_interval_std"] * 1000,
            "right_interval_mean_ms": self._offset_info["right_interval_mean"] * 1000,
            "right_interval_std_ms": self._offset_info["right_interval_std"] * 1000,
            "calibrated_offset_ms": self._offset_info["median_offset"] * 1000,
            "pipeline_retries": self.pipeline_retries
        }