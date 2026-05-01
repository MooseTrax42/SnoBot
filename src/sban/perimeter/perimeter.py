"""
Perimeter data model.

Stores a closed polygon boundary defined by odometry waypoints.
Coordinates are expressed in the odometry frame active during recording.
If odometry is reset at start, the origin is (0, 0).
"""

import json
import math
from dataclasses import dataclass, field, asdict
from typing import List, Dict, Any, Tuple, Optional


@dataclass
class Waypoint:
    """A single recorded position along the perimeter."""
    x: float            # meters, relative to start
    y: float            # meters, relative to start
    heading: float      # radians, robot heading when recorded
    distance: float     # cumulative distance traveled from first waypoint (meters)
    timestamp: float    # time.time() when recorded


@dataclass
class Perimeter:
    """
    A closed polygon perimeter defined by odometry waypoints.
    Coordinates are in the odometry frame at recording time.
    """
    waypoints: List[Waypoint]
    created_at: str             # ISO 8601 timestamp
    sample_distance_m: float    # distance interval used during recording
    total_distance_m: float     # total path length
    loop_closed: bool           # whether automatic loop closure was detected
    closure_error_m: float      # distance between last waypoint and origin
    metadata: Dict[str, Any] = field(default_factory=dict)

    @property
    def vertex_count(self) -> int:
        return len(self.waypoints)

    def length(self) -> float:
        """Total perimeter path length (sum of segment lengths)."""
        if len(self.waypoints) < 2:
            return 0.0
        total = 0.0
        for i in range(len(self.waypoints)):
            j = (i + 1) % len(self.waypoints)
            dx = self.waypoints[j].x - self.waypoints[i].x
            dy = self.waypoints[j].y - self.waypoints[i].y
            total += math.sqrt(dx * dx + dy * dy)
        return total

    def area(self) -> float:
        """Polygon area via the Shoelace formula."""
        n = len(self.waypoints)
        if n < 3:
            return 0.0
        a = 0.0
        for i in range(n):
            j = (i + 1) % n
            a += self.waypoints[i].x * self.waypoints[j].y
            a -= self.waypoints[j].x * self.waypoints[i].y
        return abs(a) / 2.0

    def centroid(self) -> Tuple[float, float]:
        """Geometric centroid of the polygon."""
        n = len(self.waypoints)
        if n == 0:
            return (0.0, 0.0)
        cx = sum(w.x for w in self.waypoints) / n
        cy = sum(w.y for w in self.waypoints) / n
        return (cx, cy)

    def bounding_box(self) -> Tuple[float, float, float, float]:
        """Returns (min_x, min_y, max_x, max_y)."""
        if not self.waypoints:
            return (0.0, 0.0, 0.0, 0.0)
        xs = [w.x for w in self.waypoints]
        ys = [w.y for w in self.waypoints]
        return (min(xs), min(ys), max(xs), max(ys))

    def contains_point(self, px: float, py: float) -> bool:
        """Test if a point is inside the perimeter (ray-casting, even-odd rule)."""
        n = len(self.waypoints)
        if n < 3:
            return False
        inside = False
        j = n - 1
        for i in range(n):
            xi, yi = self.waypoints[i].x, self.waypoints[i].y
            xj, yj = self.waypoints[j].x, self.waypoints[j].y
            if ((yi > py) != (yj > py)) and (px < (xj - xi) * (py - yi) / (yj - yi) + xi):
                inside = not inside
            j = i
        return inside

    # ------------------------------------------------------------------
    # Serialization
    # ------------------------------------------------------------------

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": "1.0",
            "created_at": self.created_at,
            "sample_distance_m": self.sample_distance_m,
            "total_distance_m": round(self.total_distance_m, 4),
            "loop_closed": self.loop_closed,
            "closure_error_m": round(self.closure_error_m, 4),
            "metadata": self.metadata,
            "waypoints": [
                {
                    "x": round(w.x, 4),
                    "y": round(w.y, 4),
                    "heading": round(w.heading, 4),
                    "distance": round(w.distance, 4),
                    "timestamp": w.timestamp,
                }
                for w in self.waypoints
            ],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "Perimeter":
        wps = [
            Waypoint(
                x=w["x"],
                y=w["y"],
                heading=w["heading"],
                distance=w["distance"],
                timestamp=w["timestamp"],
            )
            for w in data["waypoints"]
        ]
        return cls(
            waypoints=wps,
            created_at=data["created_at"],
            sample_distance_m=data["sample_distance_m"],
            total_distance_m=data["total_distance_m"],
            loop_closed=data["loop_closed"],
            closure_error_m=data["closure_error_m"],
            metadata=data.get("metadata", {}),
        )

    def save(self, filepath: str) -> None:
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, filepath: str) -> "Perimeter":
        with open(filepath, "r") as f:
            data = json.load(f)
        return cls.from_dict(data)

    def __repr__(self) -> str:
        return (
            f"Perimeter(vertices={self.vertex_count}, "
            f"distance={self.total_distance_m:.1f}m, "
            f"area={self.area():.1f}m², "
            f"closed={self.loop_closed})"
        )
