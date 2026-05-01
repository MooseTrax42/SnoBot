"""
PerimeterSet data model.

Represents one outer perimeter with optional hole perimeters, all in the
same odometry frame.
"""

from __future__ import annotations

import json
import time
from dataclasses import dataclass, field
from typing import List, Dict, Any

from .perimeter import Perimeter


@dataclass
class PerimeterSet:
    outer: Perimeter
    holes: List[Perimeter] = field(default_factory=list)
    created_at: str = field(default_factory=lambda: time.strftime("%Y-%m-%dT%H:%M:%S"))
    metadata: Dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "version": "2.0",
            "created_at": self.created_at,
            "metadata": self.metadata,
            "outer": self.outer.to_dict(),
            "holes": [h.to_dict() for h in self.holes],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PerimeterSet":
        # Backward compatibility: if a single perimeter dict is provided.
        if "outer" not in data and "waypoints" in data:
            outer = Perimeter.from_dict(data)
            return cls(outer=outer, holes=[], created_at=time.strftime("%Y-%m-%dT%H:%M:%S"))

        outer = Perimeter.from_dict(data["outer"])
        holes = [Perimeter.from_dict(h) for h in data.get("holes", [])]
        return cls(
            outer=outer,
            holes=holes,
            created_at=data.get("created_at", time.strftime("%Y-%m-%dT%H:%M:%S")),
            metadata=data.get("metadata", {}),
        )

    def save(self, filepath: str) -> None:
        with open(filepath, "w") as f:
            json.dump(self.to_dict(), f, indent=2)

    @classmethod
    def load(cls, filepath: str) -> "PerimeterSet":
        with open(filepath, "r") as f:
            data = json.load(f)
        return cls.from_dict(data)
