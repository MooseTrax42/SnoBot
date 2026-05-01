"""
Path data structures for coverage planning.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional


@dataclass
class PathPoint:
    """A single point along a path."""
    x: float
    y: float
    heading: Optional[float] = None
    is_coverage: bool = True


@dataclass
class CoveragePass:
    """
    A single coverage pass (typically a straight scanline segment).

    Passes are not automatically connected; higher-level logic can
    choose how to transition between passes.
    """
    points: List[PathPoint]
    metadata: Dict[str, Any] = field(default_factory=dict)

    def length_m(self) -> float:
        if len(self.points) < 2:
            return 0.0
        total = 0.0
        for i in range(1, len(self.points)):
            dx = self.points[i].x - self.points[i - 1].x
            dy = self.points[i].y - self.points[i - 1].y
            total += (dx * dx + dy * dy) ** 0.5
        return total


@dataclass
class CoveragePath:
    """A collection of scanline passes that cover a polygon."""
    passes: List[CoveragePass]
    line_spacing_m: float
    sweep_angle_rad: float
    metadata: Dict[str, Any] = field(default_factory=dict)

    def total_length_m(self) -> float:
        return sum(p.length_m() for p in self.passes)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": "1.0",
            "line_spacing_m": self.line_spacing_m,
            "sweep_angle_rad": self.sweep_angle_rad,
            "metadata": self.metadata,
            "passes": [
                {
                    "metadata": p.metadata,
                    "points": [
                        {
                            "x": round(pt.x, 4),
                            "y": round(pt.y, 4),
                            "heading": None if pt.heading is None else round(pt.heading, 4),
                            "is_coverage": pt.is_coverage,
                        }
                        for pt in p.points
                    ],
                }
                for p in self.passes
            ],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CoveragePath":
        passes = []
        for p in data.get("passes", []):
            pts = [
                PathPoint(
                    x=pt["x"],
                    y=pt["y"],
                    heading=pt.get("heading"),
                    is_coverage=pt.get("is_coverage", True),
                )
                for pt in p.get("points", [])
            ]
            passes.append(CoveragePass(points=pts, metadata=p.get("metadata", {})))
        return cls(
            passes=passes,
            line_spacing_m=data["line_spacing_m"],
            sweep_angle_rad=data.get("sweep_angle_rad", 0.0),
            metadata=data.get("metadata", {}),
        )
