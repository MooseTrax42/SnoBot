import cv2

import sys
from pathlib import Path

# Add project paths.
sys.path.insert(0, str(Path(__file__).parent.parent))

from sbvs.stereo.stereo_processor import StereoProcessor
from sbvs.camera.stereo_camera import StereoCamera
from sbvs.object.object_processor import ObjectProcessor

# Initialize processors with tracking parameters
proc_stereo = StereoProcessor()
proc_obj = ObjectProcessor(
    crop_factor=0.2,
    confidence_history_len=25,  # Smooth confidence over 25 frames
    min_hits=5,  # Require 5 consecutive detections to confirm track
    use_builtin_tracking=False  # Use fast IoU tracker instead of YOLO's ByteTrack
)
camera = StereoCamera(sync_tolerance_ms=15.0, max_queue=2, vpi_convert=True)

cv2.namedWindow("Depth", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Depth", 1920, 1080)

with camera:
    for fL, fR, tL, tR in camera.frames():
        result_proc = proc_stereo.run(fL, fR)
        # dist_cone = result_proc.distance_forward_cone(proc_stereo.cone_mask, 36, 348)
        # print(f"{dist_cone:.2f} m")
        
        obj_img, obj_det = proc_obj.process(fL, result_proc, render=True)
        
        proc_obj.draw_info(obj_img, obj_det)
        
        # Filter for only confirmed/stable detections for downstream tasks
        confirmed_detections = [d for d in obj_det if d.get("confirmed", False)]
        
        # Print tracking stats
        """
        if obj_det:
            print(f"Frame: {len(obj_det)} total, {len(confirmed_detections)} confirmed")
            for det in obj_det:
                track_id = det.get("track_id", "N/A")
                conf = det.get("smoothed_confidence", det["confidence"])
                confirmed = "✓" if det.get("confirmed", False) else "✗"
                print(f"  ID {track_id} | {det['class_name']} | Conf: {conf:.2f} | {confirmed}")
        """
        
        cv2.imshow("Depth", obj_img)
        
        if cv2.waitKey(1) == ord('q'):
            break
        
camera.stop()
cv2.destroyAllWindows()

print(camera.get_stats())