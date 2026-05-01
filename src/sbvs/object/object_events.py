"""
Vision event emitter.

Bridges ObjectProcessor detection output to the global EventBus.
"""

import time
from typing import List, Dict, Any, Set

from common.common_events import emit, EventType, EventPriority


def emit_detections(detections: List[Dict[str, Any]], prev_track_ids: Set[int]) -> Set[int]:
    """
    Emit detection events to the global EventBus.

    Call this once per frame after ObjectProcessor.process().

    Args:
        detections: Detection list from ObjectProcessor.process()
        prev_track_ids: Set of track_ids from the previous frame (pass empty set on first call)

    Returns:
        Current set of track_ids (pass back as prev_track_ids next frame)
    """
    current_track_ids = set()

    for det in detections:
        track_id = det.get("track_id")
        if track_id is None:
            continue

        current_track_ids.add(track_id)

        if not det.get("confirmed", False):
            continue

        distance = det.get("distance")
        if distance is None or distance <= 0:
            continue

        priority = EventPriority.HIGH if distance < 1.0 else EventPriority.NORMAL

        emit(
            EventType.OBJECT_DETECTED,
            source="sbvs.object",
            priority=priority,
            track_id=track_id,
            class_name=det.get("class_name", "unknown"),
            distance=distance,
            confidence=det.get("smoothed_confidence", det.get("confidence", 0.0)),
            bbox=det.get("bbox"),
            timestamp=time.time(),
        )

    # Emit OBJECT_LOST for tracks that disappeared.
    lost_ids = prev_track_ids - current_track_ids
    for track_id in lost_ids:
        emit(
            EventType.OBJECT_LOST,
            source="sbvs.object",
            track_id=track_id,
            timestamp=time.time(),
        )

    return current_track_ids
