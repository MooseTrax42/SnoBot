import cv2
import glob
import numpy as np
import os
import vpi
from sbvs.camera.simple_camera import SimpleCamera
from config import *

class Calibration:
    """
    Captures a set of input images and creates a calibration file compatible with OpenCV and VPI processes.
    """

    def __init__(self):
        pass

    def capture_pairs(self, amount, dir=CALIB_IMAGE_DIR):
        os.makedirs(dir, exist_ok=True)
        self.camL = SimpleCamera(CAM_ID_LEFT)
        self.camR = SimpleCamera(CAM_ID_RIGHT)

        cv2.namedWindow("Stereo Capture", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Stereo Capture", 3440, 1440)
        print("[CALIB] Press SPACE to capture an image pair, ESC to quit.")

        count = 0
        while count < amount:
            retL, frameL = self.camL.read()
            retR, frameR = self.camR.read()

            if not retL or not retR:
                print("[ERROR] Camera read failed!")
                continue

            preview = cv2.hconcat([frameL, frameR])
            cv2.imshow("Stereo Capture", preview)
            key = cv2.waitKey(1) & 0xFF

            if key == 27:
                break
            elif key == ord(' '):
                pathL = os.path.join(dir, f"left_{count:02d}.png")
                pathR = os.path.join(dir, f"right_{count:02d}.png")
                cv2.imwrite(pathL, frameL)
                cv2.imwrite(pathR, frameR)
                count += 1
                print(f"[IMAGE] Saved pair {count}: {pathL}, {pathR}")
                
        self.camL.release()
        self.camR.release()
        cv2.destroyAllWindows()

    def generate_object_points(self):
        """
        Generate a single object points array for the DIM_BOARD pattern.
        """
        objp = np.zeros((DIM_BOARD[0] * DIM_BOARD[1], 1, 3), np.float32)
        objp[:, 0, :2] = np.mgrid[0:DIM_BOARD[0], 0:DIM_BOARD[1]].T.reshape(-1, 2)
        objp *= SQUARE_SIZE
        return objp

    def calibrate_lens(self, cam, dir=CALIB_IMAGE_DIR, res=DEFAULT_RES):
        """
        Detect corners for a single lens.
        Returns image points, camera matrix, distortion, RMS, and VPI map.
        """
        image_set = sorted(glob.glob(f"{dir}/{cam}_*.png"))
        if len(image_set) == 0:
            raise RuntimeError(f"[ERROR] No calibration images found for {cam}.")

        image_points = []
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
        corner_flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE

        for image in image_set:
            img = cv2.imread(image, cv2.IMREAD_GRAYSCALE)
            
            if (img.shape[:1]) != res:
                img = cv2.resize(img, res, interpolation=cv2.INTER_LINEAR)
            if img is None:
                continue
            found, corners = cv2.findChessboardCorners(img, DIM_BOARD, flags=corner_flags)
            if found:
                corners = cv2.cornerSubPix(img, corners, (11, 11), (-1,-1), criteria)
                image_points.append(corners)
            else:
                print(f"[WARN] Checkerboard not found in {image}.")

        # Initialize camera matrices
        K = np.eye(3)
        D = np.zeros((4,1))
        calib_flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW

        objp = self.generate_object_points()
        objpoints = [objp] * len(image_points)

        rms, camMatrix, coeffs, rvecs, tvecs = cv2.fisheye.calibrate(
            objpoints, image_points, res, K, D, None, None,
            flags=calib_flags, criteria=criteria
        )
        return image_points, K, D, rms, camMatrix, coeffs, rvecs, tvecs

    def calibrate_stereo(self, objpoints, imgpointsL, imgpointsR, K_left=None, D_left=None, K_right=None, D_right=None, res=DEFAULT_RES):
        if K_left is None: K_left = np.eye(3)
        if D_left is None: D_left = np.zeros((4,1))
        if K_right is None: K_right = np.eye(3)
        if D_right is None: D_right = np.zeros((4,1))

        # Keep only valid pairs
        valid_pairs = min(len(imgpointsL), len(imgpointsR))
        objpoints = objpoints[:valid_pairs]
        imgpointsL = imgpointsL[:valid_pairs]
        imgpointsR = imgpointsR[:valid_pairs]

        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC + cv2.fisheye.CALIB_CHECK_COND + cv2.fisheye.CALIB_FIX_SKEW
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        rms, K_left, D_left, K_right, D_right, R, T, E, F = cv2.fisheye.stereoCalibrate(
            objpoints, imgpointsL, imgpointsR,
            K_left, D_left, K_right, D_right,
            res, flags=flags, criteria=criteria
        )

        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K_left, D_left, K_right, D_right, 
            res, R, T, flags=cv2.CALIB_ZERO_DISPARITY, balance=0.0
        )

        return {
            "K1": K_left, "D1": D_left,
            "K2": K_right, "D2": D_right,
            "R": R, "T": T,
            "R1": R1, "R2": R2,
            "P1": P1, "P2": P2,
            "Q": Q,
            "rmsStereo": rms
        }

    def summarize(self, data):
        """
        Prints a detailed calibration summary including VPI warp maps info.
        """
        print("\n===== Stereo Calibration Summary =====")
        print("\n--- Lens Errors (px) ---")
        print(f"Left Camera RMS Error: {data['rmsL']}")
        print(f"Right Camera RMS Error: {data['rmsR']}")
        print(f"Stereo RMS error: {data['rmsStereo']}")
        
        print("\n--- Intrinsic Parameters ---")
        print("K1 (Left Camera):\n", data["K1"])
        print("D1 (Left Camera):\n", data["D1"].flatten())
        print("K2 (Right Camera):\n", data["K2"])
        print("D2 (Right Camera):\n", data["D2"].flatten())
        
        print("\n--- Rotation & Translation ---")
        print("R (rotation between cameras):\n", data["R"])
        print("\nT (translation between cameras (mm)):\n", data["T"].flatten())
        
        print("\n--- Rectification Matrices ---")
        print("R1 (Left):\n", data["R1"])
        print("R2 (Right):\n", data["R2"])
        
        print("\n--- Projection Matrices ---")
        print("P1 (Left):\n", data["P1"])
        print("P2 (Right):\n", data["P2"])
        
        print("\n--- Rectification Information ---")
        # Print VPI map info if available
        if "vpi_mapL" in data:
            # Check for the existence of the 'grid' attribute and print size
            try:
                # The 'grid' property should exist and has a 'size' attribute (W, H)
                size_wh = data['vpi_mapL'].grid.size
                print(f"Left VPI WarpMap (Size: {size_wh[0]}x{size_wh[1]})")
                size_wh = data['vpi_mapR'].grid.size
                print(f"Right VPI WarpMap (Size: {size_wh[0]}x{size_wh[1]})")
            except AttributeError:
                # Fallback if 'grid' is missing or has no 'size'
                print(f"Left VPI WarpMap (Type: {type(data['vpi_mapL'])})")
                print(f"Right VPI WarpMap (Type: {type(data['vpi_mapR'])})")
        
        print("\n--- Disparity-to-Depth Mapping ---")
        print("Q:\n", data["Q"])
        print("\n====================================\n")


    def save(self, filename, data, rmsL=None, rmsR=None):
        """
        Saves all calibration data including OpenCV maps into a compressed npz file.
        """
        np.savez_compressed(
            filename,
            K1=data["K1"], D1=data["D1"],
            K2=data["K2"], D2=data["D2"],
            R=data["R"], T=data["T"],
            R1=data["R1"], R2=data["R2"],
            P1=data["P1"], P2=data["P2"],
            Q=data["Q"],
            rmsL=rmsL, rmsR=rmsR, rmsStereo=data["rmsStereo"]
            # No need for allow_pickle=True for numpy arrays, but doesn't hurt.
        )
        
        print(f"[INFO] Calibration data saved to {filename}")


    def calibrate(self, summarize = False, res=DEFAULT_RES):
        
        if not os.path.exists(CALIB_FILE_DIR):
            self.capture_pairs(CAP_COUNT)
        
        objp = self.generate_object_points()

        imgpointsL, K_left, D_left, rmsL, camMatrixL, coeffsL, rvecsL, tvecsL = self.calibrate_lens("left", res=res)
        imgpointsR, K_right, D_right, rmsR, camMatrixR, coeffsR, rvecsR, tvecsR = self.calibrate_lens("right", res=res)

        valid_pairs = min(len(imgpointsL), len(imgpointsR))
        object_points = [objp] * valid_pairs

        data = self.calibrate_stereo(object_points, imgpointsL, imgpointsR, K_left, D_left, K_right, D_right, res=res)
        self.save(CALIB_FILE_DIR, data, rmsL, rmsR)
        
        
        if summarize:
            self.summarize(self.load(CALIB_FILE_DIR))
        
    def load(self, filename, res=DEFAULT_RES):
        """
        Loads calibration data from a .npz file, converts OpenCV maps into VPI WarpMap objects.
        Returns a dictionary with calibration matrices, RMS, and VPI maps.
        """
        if not os.path.exists(filename):
            #raise FileNotFoundError(f"[ERROR] Calibration file {filename} does not exist.")
            self.calibrate()

        npz = np.load(filename)

        data = {k: npz[k] for k in npz.files}

        # Image size
        W, H = DEFAULT_RES
        grid = vpi.WarpGrid((W, H))

        # --- Intrinsics (2x3) ---
        K1_in = data["K1"][0:2, :]
        K2_in = data["K2"][0:2, :]

        # Optional: use rectified intrinsics from P1/P2
        K1_out = data["P1"][:2, :3]
        K2_out = data["P2"][:2, :3]

        # --- Extrinsics (3x4) ---
        X1 = np.zeros((3, 4), dtype=np.float32)
        X2 = np.zeros((3, 4), dtype=np.float32)

        X1[:, :3] = data["R1"]
        X2[:, :3] = data["R2"]
        # Translation = 0 for rectification

        # --- Distortion ---
        D1 = data["D1"].flatten()
        D2 = data["D2"].flatten()

        # --- VPI WarpMaps ---
        vpi_mapL = vpi.WarpMap.fisheye_correction(
            grid=grid,
            Kin=K1_in,
            X=X1,
            Kout=K1_out,
            coeffs=D1,
            mapping=vpi.FisheyeMapping.EQUIDISTANT
        )

        vpi_mapR = vpi.WarpMap.fisheye_correction(
            grid=grid,
            Kin=K2_in,
            X=X2,
            Kout=K2_out,
            coeffs=D2,
            mapping=vpi.FisheyeMapping.EQUIDISTANT
        )

        data["vpi_mapL"] = vpi_mapL
        data["vpi_mapR"] = vpi_mapR


        print("[INFO] Calibration data loaded (VPI-native warp maps)")

        return data

    def hasFileGenerated(self):
        if os.path.exists(CALIB_FILE_DIR):
            return True
        return False

    def build_vpi_map(self, camMatrix, coeffs, image_size=DEFAULT_RES):
        grid = vpi.WarpGrid(image_size)
        warp_map = vpi.WarpMap.fisheye_correction(
            grid,
            K=camMatrix[0:2, :],
            X=np.eye(3, 4),
            coeffs=coeffs,
            mapping=vpi.FisheyeMapping.EQUIDISTANT
        )
        return warp_map