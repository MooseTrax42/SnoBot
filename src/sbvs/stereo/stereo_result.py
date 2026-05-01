# A wrapper class for StereoProcessor for data.

import numpy as np
from dataclasses import dataclass

@dataclass(frozen=True)
class StereoResult:
    rawL: np.ndarray | None = None
    rawR: np.ndarray | None = None
    rectL: np.ndarray | None = None
    rectR: np.ndarray | None = None
    disparity: np.ndarray | None = None
    confidence: np.ndarray | None = None
    depth: np.ndarray | None = None
    
    def distance_at(self, x: int, y: int, radius: int = 0) -> float:
        """
        Returns a result in meters.
        """
        h, w = self.depth.shape
        x0, x1 = max(0, x - radius), min(w, x + radius + 1)
        y0, y1 = max(0, y - radius), min(h, y + radius + 1)
        region = self.depth[y0:y1, x0:x1]
        valid = region[region > 0]
        
        # Meters plus a factor of two due to no fp division before creating the depth map.
        return float(np.median(valid)) / 500 if valid.size else 0.0
    
    def distance_roi(self, x1: int, y1: int, x2: int, y2: int, downscale_factor: int | float = 1, crop_factor: float = 0.0) -> float:
        h, w = self.depth.shape
        
        if crop_factor > 0:
            width = x2 - x1
            height = y2 - y1
            crop_x = int(width * crop_factor)
            crop_y = int(height * crop_factor)
            
            x1 += crop_x
            y1 += crop_y
            x2 -= crop_x
            y2 -= crop_y
        
        x_min = max(0, int(x1 / downscale_factor))
        x_max = min(w, int(x2 / downscale_factor))
        y_min = max(0, int(y1 / downscale_factor))
        y_max = min(h, int(y2 / downscale_factor))
        
        # 2. Extract the region of interest directly
        region = self.depth[y_min:y_max, x_min:x_max]
        
        # 3. Filter for valid depth readings (those greater than 0)
        valid = region[region > 0]
        return float(np.median(valid)) / 500 if valid.size else 0.0
    
    def distance_forward_cone(self, cone_mask, v_min=0, v_max=None, percentile=5):
        depth = self.depth
        h, w = depth.shape
        if v_max is None:
            v_max = h

        region = depth[v_min:v_max, cone_mask]
        valid = region[region > 0]

        if valid.size == 0:
            return 0.0

        return float(np.percentile(valid, percentile)) / 500
