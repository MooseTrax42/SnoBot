import cv2
import numpy as np

from typing import Optional, Tuple

from sbvs.camera.pipeline import simple_pipeline


class SimpleCamera:
    """ 
    A non-threaded, simple camera class for calibration and preview.
    Reads frames on demand via the read() method.
    """    
    def __init__(self, sensor_id: int, out_format: str = "BGR"):
        self.sensor_id = sensor_id
        self.out_format = out_format
        self.pipeline = simple_pipeline(sensor_id, out_format)
        self.cap = cv2.VideoCapture(self.pipeline, cv2.CAP_GSTREAMER)
        
        if not self.cap.isOpened():
            raise RuntimeError(f"[SIMPLECAM] Failed to open camera {sensor_id} with pipeline: \n{self.pipeline}")
        
    def read(self) -> Tuple[bool, Optional[np.ndarray]]:
        """Reads the lastest frame from the GStreamer pipeline."""
        return self.cap.read()
    
    def release(self):
        """Releases the OpenCV capture resource."""
        if self.cap is not None:
            self.cap.release()
            self.cap = None
            
    # Context manager for easy resource management.
    def __enter__(self):
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.release()