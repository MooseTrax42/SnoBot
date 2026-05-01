# Handling rectification, disparity, and depth.

import numpy as np
import vpi
import time

from config import CALIB_FILE_DIR, CAM_RES_RAW, CAM_RES_DS, MAX_SCALED_DISP, MAX_DEPTH, DISP_WINDOW_SIZE, DISP_MAX, CONF_THRESHOLD, SAFETY_CONE_DEG
from sbvs.calibration import Calibration
from sbvs.stereo.stereo_result import StereoResult

def make_forward_cone_mask(width, fx, cx, theta_deg):
    theta = np.deg2rad(theta_deg)
    u = np.arange(width)
    angles = np.arctan((u - cx) / fx)
    return np.abs(angles) < theta


class StereoProcessor:
    """  
    A class to perform image rectification and stereo disparity estimation using Nvidia VPI for maximum efficiency.\n
    Consists of pipeline methods and a public run() to return data.
    """
    
    def __init__(self, calib_file=CALIB_FILE_DIR, res=CAM_RES_DS):
        print("[INIT] Loading calibration file and VPI resources...")
        self.width, self.height = res
        self.img_size = (self.width, self.height)
        
        # Load calibration data.
        self.calib = Calibration()
        self.calibration = self.calib.load(calib_file)
        self.vpi_mapL = self.calibration["vpi_mapL"]
        self.vpi_mapR = self.calibration["vpi_mapR"]
        
        # Depth values.
        self.Q = self.calibration["Q"]
        self.Q23 = self.Q[2, 3]
        self.Q32 = self.Q[3, 2]
        self.Q33 = self.Q[3, 3]
        
        # Synthetic camera parameters for safety cone fallback. 
        self.fx = self.Q23
        self.cx = self.Q[0, 3]
        self.cone_mask = make_forward_cone_mask(self.width, self.fx, self.cx, SAFETY_CONE_DEG)
        
        # Pre-allocate VPI buffers.
        self.vpi_rectL = vpi.Image(self.img_size, vpi.Format.Y8_ER)
        self.vpi_rectR = vpi.Image(self.img_size, vpi.Format.Y8_ER)
        self.vpi_disparity = vpi.Image(self.img_size, vpi.Format.S16)
        self.vpi_confidence = vpi.Image(self.img_size, vpi.Format.U16)
        
        # Input image space.
        self.vpi_inputL = vpi.Image(self.img_size, vpi.Format.U8)
        self.vpi_inputR = vpi.Image(self.img_size, vpi.Format.U8)
        
        # Tracking for FPS calculation.
        self.frame_count = 0
        self.start_time = time.time()

        print(f"[INIT] Processor ready. Resolution: {self.width}x{self.height}")

    def process(self, frameL: vpi.Image, frameR: vpi.Image) -> "StereoProcessor":
        """  
        Part of the image processing pipeline. Stores and rescales raw images.
        """
        
        with vpi.Backend.CUDA:
            self.vpi_inputL = frameL.convert(vpi.Format.U8).rescale(self.img_size, interp=vpi.Interp.LINEAR, border=vpi.Border.ZERO)
            self.vpi_inputR = frameR.convert(vpi.Format.U8).rescale(self.img_size, interp=vpi.Interp.LINEAR, border=vpi.Border.ZERO)
        
        self.frame_count += 1
        return self
    
    def rectify(self, use_clahe: bool = False, use_gauss: bool = False) -> "StereoProcessor":
        """  
        Part of the image processing pipeline. Applies WarpMaps to rectify distorted images.
        """
        
        if self.vpi_inputL is None or self.vpi_inputR is None:
            raise RuntimeError("Must call process() first.")
    
        with vpi.Backend.CUDA: 
            
            # Expects U8 format. Documentation available at: 
            self.vpi_rectL = self.vpi_inputL.remap(wmap=self.vpi_mapL, interp=vpi.Interp.CATMULL_ROM, border=vpi.Border.ZERO)
            self.vpi_rectR = self.vpi_inputR.remap(wmap=self.vpi_mapR, interp=vpi.Interp.CATMULL_ROM, border=vpi.Border.ZERO)
            
            # Constrast enhancement through histogram.
            if use_clahe:
                self.vpi_rectL = self.vpi_rectL.eqhist()
                self.vpi_rectR = self.vpi_rectR.eqhist()
                
            # Gaussian blur pre-filter.
            if use_gauss:
                self.vpi_rectL = self.vpi_rectL.gaussian_filter(7, 1.7, border=vpi.Border.ZERO)
                self.vpi_rectR = self.vpi_rectR.gaussian_filter(7, 1.7, border=vpi.Border.ZERO)
            
            # Format to pass to disparity.
            self.vpi_rectL = self.vpi_rectL.convert(vpi.Format.Y8_ER)
            self.vpi_rectR = self.vpi_rectR.convert(vpi.Format.Y8_ER)
            
        return self
    
    def disparity(self, use_bilateral: bool = False, use_gauss: bool = False) -> "StereoProcessor":
        """
        Part of the image processing pipeline. Uses rectified image pairs to create a disparity map.
        """
        
        if not self.vpi_rectL or not self.vpi_rectR:
            raise RuntimeError("Must call rectify() first.")
        
        # Compute disparity map.
        with vpi.Backend.CUDA:
            vpi_disp = vpi.stereodisp(self.vpi_rectL, self.vpi_rectR, out_confmap=self.vpi_confidence, window=DISP_WINDOW_SIZE, maxdisp=DISP_MAX)
            self.vpi_disparity = vpi_disp.convert(vpi.Format.S16)
        
            if use_bilateral:
                self.vpi_disparity = self.vpi_disparity.bilateral_filter(7, 11, 90, border=vpi.Border.ZERO)
            
            if use_gauss:
                self.vpi_disparity = self.vpi_disparity.gaussian_filter(7, 1.7, border=vpi.Border.ZERO)
            
        return self
    
    def depth(self):
        disp = self.vpi_disparity.cpu().astype(np.float32) / 16.0
        conf = self.vpi_confidence.cpu().astype(np.float32) / 65535.0
        denom = disp * self.Q32 + self.Q33
        denom[np.abs(denom) < 1e-6] = 1e-6
        
        depth = self.Q23 / denom
        
        NORM_THRESH = CONF_THRESHOLD / 65535.0
        invalid = (disp < 0.1) | (conf < NORM_THRESH)
        depth[invalid] = 0.0
        
        return depth, disp, conf

    def run(self, frameL: vpi.Image, frameR: vpi.Image, *, clahe: bool = False, in_gauss: bool = False, bilateral: bool = False, out_gauss: bool = False) -> StereoResult:
        """  
        Executes the full stereo pipeline and returns numeric results.
        """
        
        t0 = time.perf_counter()
        
        self.process(frameL, frameR)
        self.rectify(clahe, in_gauss)
        self.disparity(bilateral, out_gauss)
        depth, disp, conf = self.depth()
        
        t1 = time.perf_counter()
        proc_ms = (t1 - t0) * 1000
        # print(f"[PROC] Stereo depth time: {proc_ms:.2f} ms")
        
        return StereoResult(rawL = None, rawR = None, rectL = None, rectR = None,
                disparity=disp, confidence=conf, depth=depth)