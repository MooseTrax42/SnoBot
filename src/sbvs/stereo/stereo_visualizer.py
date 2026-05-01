# Class for returning data in image format.

import cv2
import numpy as np

from sbvs.stereo.stereo_result import StereoResult
from config import MAX_SCALED_DISP, MAX_DEPTH

class StereoVisualizer:
    """
    Turns a StereoResult into different image formats intended for human interpretation.
    """
    
    def __init__(self, result: StereoResult):
        self.r = result
       
    def rectified_pair(self) -> np.ndarray:
        l = cv2.cvtColor(self.r.rectL, cv2.COLOR_GRAY2BGR)
        r = cv2.cvtColor(self.r.rectR, cv2.COLOR_GRAY2BGR)
        return cv2.hconcat([l, r])
    
    def disparity(self) -> np.ndarray:
        disp = np.clip(self.r.disparity, 0, MAX_SCALED_DISP)
        disp_vis = cv2.normalize(disp, None, 0, 255, cv2.NORM_MINMAX)
        disp_vis = disp_vis.astype(np.uint8)
        return cv2.applyColorMap(disp_vis, cv2.COLORMAP_JET)

    def confidence(self) -> np.ndarray:
        conf = np.clip(self.r.confidence * 255.0, 0, 255).astype(np.uint8)
        return cv2.cvtColor(conf, cv2.COLOR_GRAY2BGR)

    def depth(self) -> np.ndarray:
        depth = np.clip(self.r.depth, 0, MAX_DEPTH)
        norm = (1.0 - depth / MAX_DEPTH) * 255.0
        return cv2.cvtColor(norm.astype(np.uint8), cv2.COLOR_GRAY2BGR)

    def full_debug_view(self) -> np.ndarray:
        top = cv2.hconcat([self.rectified_pair(), self.depth()])
        bottom = cv2.hconcat([self.disparity(), self.confidence()])
        return cv2.vconcat([top, bottom])
    