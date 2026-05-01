import cv2
import vpi
import time
import numpy as np
from collections import defaultdict, deque

from ultralytics import YOLO

from sbvs.stereo.stereo_result import StereoResult
from config import OBJ_ENGINE_DIR, DOWNSCALE_FACTOR

class SimpleTracker:
    """Lightweight IoU-based tracker for fast object association."""
    
    def __init__(self, iou_threshold: float = 0.3, max_age: int = 5, spatial_hash_bins: int = 4):
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.next_id = 0
        self.tracks = {}  # {track_id: {"bbox": (x1,y1,x2,y2), "age": int, "class_id": int}}
        self.spatial_hash_bins = spatial_hash_bins  # Grid bins for spatial hashing
        
    @staticmethod
    def iou(box1, box2):
        """Calculate IoU between two boxes (x1, y1, x2, y2). Optimized version."""
        x1_1, y1_1, x2_1, y2_1 = box1
        x1_2, y1_2, x2_2, y2_2 = box2
        
        # Early exit: check if boxes can possibly overlap
        if x2_1 < x1_2 or x2_2 < x1_1 or y2_1 < y1_2 or y2_2 < y1_1:
            return 0.0
        
        xi1 = max(x1_1, x1_2)
        yi1 = max(y1_1, y1_2)
        xi2 = min(x2_1, x2_2)
        yi2 = min(y2_1, y2_2)
        
        inter_area = (xi2 - xi1) * (yi2 - yi1)
        box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
        box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
        union_area = box1_area + box2_area - inter_area
        
        return inter_area / union_area
    
    def _get_spatial_bin(self, bbox, frame_width=1920, frame_height=1080):
        """Assign bbox to spatial grid bin for faster candidate filtering."""
        x1, y1, x2, y2 = bbox
        cx = (x1 + x2) / 2
        cy = (y1 + y2) / 2
        
        bin_x = int(cx / frame_width * self.spatial_hash_bins)
        bin_y = int(cy / frame_height * self.spatial_hash_bins)
        
        # Clamp to valid range
        bin_x = min(bin_x, self.spatial_hash_bins - 1)
        bin_y = min(bin_y, self.spatial_hash_bins - 1)
        
        return (bin_x, bin_y)
    
    def _get_neighbor_bins(self, bin_coords):
        """Get neighboring bins including diagonal neighbors."""
        bin_x, bin_y = bin_coords
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                nx = bin_x + dx
                ny = bin_y + dy
                if 0 <= nx < self.spatial_hash_bins and 0 <= ny < self.spatial_hash_bins:
                    neighbors.append((nx, ny))
        return neighbors
    
    def update(self, detections):
        """
        Update tracks with new detections using spatial hashing for speed.
        
        Args:
            detections: List of dicts with "bbox" and "class_id"
            
        Returns:
            List of track_ids corresponding to each detection
        """
        if not detections:
            # Age out tracks
            dead_tracks = [tid for tid, t in self.tracks.items() if t["age"] >= self.max_age]
            for tid in dead_tracks:
                del self.tracks[tid]
            for t in self.tracks.values():
                t["age"] += 1
            return []
        
        track_ids = [-1] * len(detections)
        
        # Build spatial hash of existing tracks
        track_spatial_map = defaultdict(list)
        for track_id, track in self.tracks.items():
            bin_coords = self._get_spatial_bin(track["bbox"])
            track_spatial_map[bin_coords].append(track_id)
        
        # Match detections to existing tracks
        matched_tracks = set()
        
        for i, det in enumerate(detections):
            det_bbox = det["bbox"]
            det_class = det["class_id"]
            det_bin = self._get_spatial_bin(det_bbox)
            
            # Only check tracks in nearby spatial bins
            candidate_track_ids = []
            for neighbor_bin in self._get_neighbor_bins(det_bin):
                candidate_track_ids.extend(track_spatial_map.get(neighbor_bin, []))
            
            best_iou = self.iou_threshold
            best_track_id = None
            
            for track_id in candidate_track_ids:
                if track_id in matched_tracks:
                    continue
                    
                track = self.tracks[track_id]
                
                # Only match same class
                if track["class_id"] != det_class:
                    continue
                
                iou = self.iou(det_bbox, track["bbox"])
                if iou > best_iou:
                    best_iou = iou
                    best_track_id = track_id
            
            if best_track_id is not None:
                # Update existing track
                self.tracks[best_track_id]["bbox"] = det_bbox
                self.tracks[best_track_id]["age"] = 0
                track_ids[i] = best_track_id
                matched_tracks.add(best_track_id)
            else:
                # Create new track
                new_id = self.next_id
                self.next_id += 1
                self.tracks[new_id] = {
                    "bbox": det_bbox,
                    "age": 0,
                    "class_id": det_class
                }
                track_ids[i] = new_id
        
        # Age out unmatched tracks
        dead_tracks = []
        for track_id, track in self.tracks.items():
            if track_id not in matched_tracks:
                track["age"] += 1
                if track["age"] >= self.max_age:
                    dead_tracks.append(track_id)
        
        for tid in dead_tracks:
            del self.tracks[tid]
        
        return track_ids


class ObjectProcessor:
    def __init__(self, engine: str = OBJ_ENGINE_DIR, task: str = "detect", 
                 downscale_factor: int | float = DOWNSCALE_FACTOR, 
                 crop_factor: float = 0.2,
                 confidence_history_len: int = 10,
                 min_hits: int = 3,
                 use_builtin_tracking: bool = False):
        """
        Args:
            engine (str): TensorRT input engine. Defaults to OBJ_ENGINE_DIR.
            task (str): 'detect', 'segment', 'classify', 'pose', or 'obb'. Defaults to "detect".
            downscale_factor (int | float): Proper frame scaling. Defaults to DOWNSCALE_FACTOR.
            crop_factor (float): Crop factor for distance calculation.
            confidence_history_len (int): Number of frames to keep for confidence smoothing.
            min_hits (int): Minimum consecutive detections before marking track as "confirmed".
            use_builtin_tracking (bool): Use YOLO's built-in tracking (slower) vs simple IoU tracker (faster).
        """
        self.engine = engine
        self.task = task
        self.model = YOLO(engine, task)
        self.downscale_factor = downscale_factor
        self.crop_factor = crop_factor
        self.use_builtin_tracking = use_builtin_tracking
        
        # Tracking confidence management
        self.confidence_history_len = confidence_history_len
        self.min_hits = min_hits
        
        # Track history: {track_id: deque of confidences}
        self.track_confidences = defaultdict(lambda: deque(maxlen=confidence_history_len))
        # Track hit counts: {track_id: consecutive_frames_seen}
        self.track_hits = defaultdict(int)
        # Track ages: {track_id: total_frames_tracked}
        self.track_ages = defaultdict(int)
        
        # Simple tracker for fast tracking
        if not use_builtin_tracking:
            self.tracker = SimpleTracker(iou_threshold=0.3, max_age=5)
        
    def _update_track_confidence(self, track_id: int, confidence: float):
        """Update confidence history for a track."""
        self.track_confidences[track_id].append(confidence)
        self.track_hits[track_id] += 1
        self.track_ages[track_id] += 1
        
    def _get_smoothed_confidence(self, track_id: int) -> float:
        """Get exponentially weighted moving average of confidence."""
        if track_id not in self.track_confidences:
            return 0.0
        
        confidences = list(self.track_confidences[track_id])
        if not confidences:
            return 0.0
        
        # Exponential weighting: more recent = higher weight
        weights = [0.5 ** (len(confidences) - i - 1) for i in range(len(confidences))]
        weighted_sum = sum(c * w for c, w in zip(confidences, weights))
        weight_sum = sum(weights)
        
        return weighted_sum / weight_sum if weight_sum > 0 else 0.0
    
    def _is_track_confirmed(self, track_id: int) -> bool:
        """Check if track has been seen enough times to be considered stable."""
        return self.track_hits[track_id] >= self.min_hits
    
    def cleanup_stale_tracks(self, active_track_ids: set):
        """Remove tracks that are no longer active."""
        all_tracked = set(self.track_hits.keys())
        stale_tracks = all_tracked - active_track_ids
        
        for track_id in stale_tracks:
            del self.track_confidences[track_id]
            del self.track_hits[track_id]
            del self.track_ages[track_id]
        
    def process(self, vpi_frame: vpi.Image, stereo_result: StereoResult = None, 
                verbose: bool = False, render: bool = False, 
                timing: dict | None = None, conf_threshold: float = 0.35):
        """
        Process frame with object detection and tracking.
        
        Args:
            conf_threshold: Confidence threshold for detections (lower = more detections but more false positives)
        """
        t_start = time.monotonic()

        frame = vpi_frame.cpu()
        
        # Use predict (fast) with confidence threshold
        results = self.model.predict(frame, device=0, verbose=False, conf=conf_threshold)
        yolo_result = results[0]

        # Build initial detections from YOLO output
        detections = []
        boxes = yolo_result.boxes
        
        # Vectorized extraction (faster than loop for many objects)
        if len(boxes) > 0:
            xyxy = boxes.xyxy.cpu().numpy().astype(int)
            cls_ids = boxes.cls.cpu().numpy().astype(int)
            confs = boxes.conf.cpu().numpy()
            
            for i in range(len(boxes)):
                x1, y1, x2, y2 = xyxy[i]
                cls_id = cls_ids[i]
                conf = float(confs[i])
                class_name = self.model.names[cls_id]

                detection = {
                    "class_id": cls_id,
                    "class_name": class_name,
                    "confidence": conf,
                    "bbox": (x1, y1, x2, y2)
                }
                detections.append(detection)
        
        # Apply tracking
        if not self.use_builtin_tracking and detections:
            track_ids = self.tracker.update(detections)
            active_track_ids = set(tid for tid in track_ids if tid >= 0)
            
            # Add tracking info to detections
            for det, track_id in zip(detections, track_ids):
                if track_id >= 0:
                    self._update_track_confidence(track_id, det["confidence"])
                    det["track_id"] = track_id
                    det["smoothed_confidence"] = self._get_smoothed_confidence(track_id)
                    det["confirmed"] = self._is_track_confirmed(track_id)
                    det["track_age"] = self.track_ages[track_id]
            
            # Cleanup stale tracks
            self.cleanup_stale_tracks(active_track_ids)
        elif not self.use_builtin_tracking:
            # No detections, update tracker
            self.tracker.update([])
        
        # Add distance if stereo available (only for detections with distance info needed)
        if stereo_result is not None:
            for det in detections:
                x1, y1, x2, y2 = det["bbox"]
                distance = stereo_result.distance_roi(x1, y1, x2, y2, 
                                                     self.downscale_factor, 
                                                     self.crop_factor)
                det["distance"] = distance
        
        # Render if requested (defer to end to avoid blocking)
        annotated_frame = yolo_result.plot() if render else None

        t_end = time.monotonic()
        if timing is not None:
            timing["object_latency_ms"] = (t_end - t_start) * 1000.0

        return annotated_frame, detections

 
    @staticmethod 
    def draw_info(frame, detections, start_x=50, start_y=50, line_height=40,
                  font=cv2.FONT_HERSHEY_SIMPLEX, font_scale=1.0,
                  color=(255, 255, 255), thickness=2,
                  show_confidence=False, show_track_id=True):
        """
        Draws detection info on frame.
        
        - Green color for confirmed tracks
        - White for unconfirmed tracks
        """
        for i, det in enumerate(detections):
            # Determine label
            label = det["class_name"]

            if show_confidence:
                # Show smoothed confidence if available, otherwise raw confidence
                conf_val = det.get("smoothed_confidence", det["confidence"])
                label += f" ({conf_val:.2f})"

            if "distance" in det:
                label += f" {det['distance']:.2f}m"

            if show_track_id and "track_id" in det:
                label = f"ID {det['track_id']} | {label}"

            # Choose color: green if confirmed, else default
            text_color = (0, 255, 0) if det.get("confirmed", True) else color

            y_pos = start_y + i * line_height
            cv2.putText(frame, label, (start_x, y_pos), font, font_scale, 
                       text_color, thickness)