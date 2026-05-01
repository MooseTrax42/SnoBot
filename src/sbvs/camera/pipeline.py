from config import CAM_WIDTH_RAW, CAM_HEIGHT_RAW, CAM_FPS_RAW

# GStreamer pipeline builder with selectable output formats.
@DeprecationWarning
def combined_pipeline(left_id: int, right_id: int, out_format: str = "BGR", width: int = CAM_WIDTH_RAW, height: int = CAM_HEIGHT_RAW, fps: int = CAM_FPS_RAW) -> str:
    convert_map = {
        "BGR": "BGR",
        "BGRx": "BGRx",
        "GRAY8": "GRAY8",
        "RGBA": "RGBA",
        "NV12": "NV12"
    }
    
    if out_format not in convert_map:
        raise ValueError(f"[FORMAT] Unsupported format {out_format}. Supported: {list(convert_map.keys())}")

    return (
        "multiqueue name=mq max-size-buffers=1 sync-by-running-time=true "
        # Left camera
        f"nvarguscamerasrc sensor-id={left_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
        "mq.sink_0 "
        # Right camera
        f"nvarguscamerasrc sensor-id={right_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
        "mq.sink_1 "
        # Left output
        "mq.src_0 ! nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format={out_format} ! "
        "appsink name=left drop=1 max-buffers=1 sync=true "
        # Right output
        "mq.src_1 ! nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format={out_format} ! "
        "appsink name=right drop=1 max-buffers=1 sync=true"
    )
@DeprecationWarning
def dual_pipeline(out_format="BGRx", width=1920, height=1080, fps=30) -> str:
    return (
        "multiqueue name=mq max-size-buffers=2 max-size-time=0 max-size-bytes=0 sync-by-running-time=true "
        f"nvarguscamerasrc sensor-id=0 do-timestamp=true ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},format=NV12,framerate={fps}/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! mq.sink_0 "
        f"nvarguscamerasrc sensor-id=1 do-timestamp=true ! "
        f"video/x-raw(memory:NVMM),width={width},height={height},format=NV12,framerate={fps}/1 ! "
        "queue max-size-buffers=1 leaky=downstream ! mq.sink_1 "
        # Use BGRx as intermediate (your original approach, which works)
        "mq.src_0 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink0 emit-signals=true sync=false max-buffers=1 drop=true "
        "mq.src_1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert ! video/x-raw,format=BGR ! "
        "appsink name=appsink1 emit-signals=true sync=false max-buffers=1 drop=true"
    )


def gstreamer_pipeline(sensor_id: int, out_format: str = "BGR", width: int = CAM_WIDTH_RAW, height: int = CAM_HEIGHT_RAW, fps: int = CAM_FPS_RAW) -> str:
    """
    Build a nvarguscamerasrc -> nvvidconv -> appsink pipeline.
    Args:
        sensor_id (int): Camera id number (0 or 1).\n
        out_format (str): BGR, BGRx, GRAY8, RGBA, or NV12.\n
        width (int): CAM_WIDTH_RAW or specified.\n
        height (int): CAM_HEIGHT_RAW or specified.\n
        fps (int): CAM_FPS_RAW or specified.
    Returns:
        A constructed pipeline. 
    """
    convert_map = {
        "BGR": "BGR",
        "BGRx": "BGRx",
        "GRAY8": "GRAY8",
        "RGBA": "RGBA",
        "NV12": "NV12"
    }
    
    if out_format not in convert_map:
        raise ValueError(f"[FORMAT] Unsupported format {out_format}. Supported: {list(convert_map.keys())}")

        '''
        f"nvarguscamerasrc sensor-id={sensor_id} "
        "bufapi-version=true "
        #"maxperf=true "
        "timeout=0 "
        "! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format={out_format} ! "
        "appsink drop=1 max-buffers=1 sync=false"
        '''
    return (
        f"nvarguscamerasrc enableFrameSync=true sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format={out_format} ! "
        "appsink drop=1 max-buffers=1 sync=true"
    )
    
def simple_pipeline(sensor_id: int, out_format: str = "BGR", width: int = CAM_WIDTH_RAW, height: int = CAM_HEIGHT_RAW, fps: int = CAM_FPS_RAW) -> str:
    convert_map = {
        "BGR": "BGR",
        "BGRx": "BGRx",
        "GRAY8": "GRAY8",
        "RGBA": "RGBA",
        "NV12": "NV12"
    }
    
    if out_format not in convert_map:
        raise ValueError(f"[FORMAT] Unsupported format {out_format}. Supported: {list(convert_map.keys())}")
    
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={width}, height={height}, format=NV12, framerate={fps}/1 ! "
        "nvvidconv ! video/x-raw, format=BGRx ! "
        f"videoconvert ! video/x-raw, format={out_format} ! "
        "appsink drop=1 max-buffers=2"
    )