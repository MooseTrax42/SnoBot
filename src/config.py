# Config file.

#|==============================|
#|     BLANK HEADING BLOCK?     |
#|==============================|


#|==============================|
#|       STREAM SETTINGS        |
#|==============================|

# Raw stream parameters.
CAM_WIDTH_RAW = 1920
CAM_HEIGHT_RAW = 1080
CAM_FPS_RAW = 25
CAM_RES_RAW = (CAM_WIDTH_RAW, CAM_HEIGHT_RAW)
CAM_MAX_FAIL = 10

# Component identification.
CAM_ID_LEFT = 1
CAM_ID_RIGHT = 0

# CameraThread timings.
THREAD_BARRIER_TIMEOUT = 5.0
THREAD_READ_FAIL_SLEEP = 0.001
THREAD_ERROR_SLEEP = 0.01
THREAD_JOIN_TIMEOUT = 2.0

# StereoCamera timings.
STEREOCAM_CALIBRATION_SAMPLES = 100
STEREOCAM_CALIBRATION_TIMEOUT = 5.0
STEREOCAM_CALIBRATION_SLEEP = 0.01
STEREOCAM_DELIVERY_SLEEP = 0.0005
STEREOCAM_GENERATOR_SLEEP = 0.0001
STEREOCAM_STARTUP_DELAY = 2.0
STEREOCAM_STOP_GRACEFUL = 0.01
STEREOCAM_JOIN_TIMEOUT = 2.0
STEREOCAM_PAIR_QUEUE_SIZE = 2
STEREOCAM_GENERATOR_QUEUE_SIZE = 2
STEREOCAM_OFFSET_TOLERANCE = 0.0085

#|==============================|
#|      DISP/DEPTH SETTINGS     |
#|==============================|

# Input setting.
DOWNSCALE_FACTOR = 3
CAM_WIDTH_DS = CAM_WIDTH_RAW // DOWNSCALE_FACTOR
CAM_HEIGHT_DS = CAM_HEIGHT_RAW // DOWNSCALE_FACTOR
CAM_RES_DS = (CAM_WIDTH_DS, CAM_HEIGHT_DS)

# Disparity settings.
DISP_WINDOW_SIZE = 7    # Must be odd.
DISP_MAX = 128          # Must be a multiple of 16.

MAX_SCALED_DISP = 256   # For proper limiting. 256 for use in StereoVisualizer otherwise 4096 for old system.
DENOISE_RADIUS = 5      # Pixels.
DENOISE_MAX_DIFF = 16   # Intensity.

CONF_THRESHOLD = 16384  # Value out of 65535 (max confidence).

MAX_DEPTH = 10000.0    # In mm.
SAFETY_CONE_DEG = 35

#|==============================|
#|        MODE SELECTION        |
#|==============================|   

# Resolution mode.  
DEFAULT_RES = CAM_RES_DS

#|==============================|
#|     CALIBRATION SETTINGS     |
#|==============================|

# Handy file locations.
CALIB_IMAGE_DIR = "data/sbvs/calibration/images"
CALIB_FILE_DIR = f"data/sbvs/calibration/stereo_{CAM_HEIGHT_DS}p.npz"

# Testing stuff.
TEST_CALIB_IMAGE_DIR = "data/test/images"
TEST_CALIB_FILE_DIR = "data/test/test.npz"

# Raw image pairs to get.
CAP_COUNT = 30

# Board parameters.
DIM_BOARD = (8, 6)  # Inner corners.
SQUARE_SIZE = 25.4  # In mm.




# Object stuff
OBJ_ENGINE_DIR = "data/sbvs/yolo12n.engine"