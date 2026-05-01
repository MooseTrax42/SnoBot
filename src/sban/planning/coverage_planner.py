"""
Scanline (boustrophedon) coverage planner.

Supports outer polygons with optional hole polygons. The planner returns
independent scanline passes; it does not attempt to connect passes with
travel segments (to avoid crossing holes).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Iterable, List, Tuple, Optional

from sban.perimeter.perimeter import Perimeter
from .path_data import CoveragePath, CoveragePass, PathPoint

try:
    from shapely.geometry import Polygon, MultiPolygon
except Exception:  # pragma: no cover - optional dependency
    Polygon = None
    MultiPolygon = None
    _HAS_SHAPELY = False
else:
    _HAS_SHAPELY = True


Point = Tuple[float, float]
Segment = Tuple[float, float]


@dataclass
class PlannerConfig:
    line_spacing_m: float = 0.6
    sweep_angle_rad: float = 0.0
    min_segment_length_m: float = 0.05
    y_epsilon: float = 1e-6
    max_scanlines: int = 12000
    max_passes: int = 20000
    pattern: str = "scanline"  # scanline | hybrid | contour
    edge_passes: int = 2
    edge_spacing_m: Optional[float] = None


class CoveragePlanner:
    """Generate scanline coverage passes within a polygon (minus holes)."""

    def __init__(self, config: Optional[PlannerConfig] = None):
        self.config = config or PlannerConfig()

    def plan(
        self,
        perimeter: Perimeter,
        holes: Optional[List[Perimeter]] = None,
    ) -> CoveragePath:
        if self.config.line_spacing_m <= 0.0:
            raise ValueError("line_spacing_m must be > 0")

        outer = _perimeter_to_polygon(perimeter)
        hole_polys = [_perimeter_to_polygon(h) for h in (holes or [])]

        pattern = (self.config.pattern or "scanline").strip().lower()
        if pattern not in ("scanline", "hybrid", "contour"):
            pattern = "scanline"

        if pattern == "scanline":
            passes = _scanline_passes(outer, hole_polys, self.config)
            metadata = {
                "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "outer_vertices": len(outer),
                "hole_count": len(hole_polys),
                "pattern": "scanline",
            }
            return CoveragePath(
                passes=passes,
                line_spacing_m=self.config.line_spacing_m,
                sweep_angle_rad=self.config.sweep_angle_rad,
                metadata=metadata,
            )

        if not _HAS_SHAPELY:
            raise RuntimeError("Hybrid/contour planning requires shapely>=2.0 to be installed")

        if self.config.edge_spacing_m is None:
            spacing = max(self.config.line_spacing_m, 0.05)
        else:
            spacing = self.config.edge_spacing_m
        if spacing <= 0:
            raise ValueError("edge spacing must be > 0")

        poly = _make_shapely_polygon(outer, hole_polys)
        if poly.is_empty:
            return CoveragePath(
                passes=[],
                line_spacing_m=self.config.line_spacing_m,
                sweep_angle_rad=self.config.sweep_angle_rad,
                metadata={"created_at": time.strftime("%Y-%m-%dT%H:%M:%S"), "pattern": pattern, "empty": True},
            )

        edge_passes = max(0, int(self.config.edge_passes))
        contour_passes: List[CoveragePass] = []
        geom = poly
        reverse = False
        for idx in range(edge_passes):
            contour_passes.extend(_contour_passes(geom, reverse=reverse, offset_index=idx))
            reverse = not reverse
            geom = geom.buffer(-spacing, join_style=2, cap_style=2)
            if geom.is_empty:
                break

        if pattern == "contour":
            metadata = {
                "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
                "outer_vertices": len(outer),
                "hole_count": len(hole_polys),
                "pattern": "contour",
                "edge_passes": edge_passes,
                "edge_spacing_m": spacing,
            }
            return CoveragePath(
                passes=contour_passes,
                line_spacing_m=self.config.line_spacing_m,
                sweep_angle_rad=self.config.sweep_angle_rad,
                metadata=metadata,
            )

        scanline_passes: List[CoveragePass] = []
        if geom and not geom.is_empty:
            for core_poly in _iter_polygons(geom):
                core_outer = _drop_closure(list(core_poly.exterior.coords))
                core_holes = [_drop_closure(list(r.coords)) for r in core_poly.interiors]
                if len(core_outer) >= 3:
                    scanline_passes.extend(_scanline_passes(core_outer, core_holes, self.config))

        metadata = {
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
            "outer_vertices": len(outer),
            "hole_count": len(hole_polys),
            "pattern": "hybrid",
            "edge_passes": edge_passes,
            "edge_spacing_m": spacing,
        }
        return CoveragePath(
            passes=contour_passes + scanline_passes,
            line_spacing_m=self.config.line_spacing_m,
            sweep_angle_rad=self.config.sweep_angle_rad,
            metadata=metadata,
        )


def _perimeter_to_polygon(perimeter: Perimeter) -> List[Point]:
    pts = [(wp.x, wp.y) for wp in perimeter.waypoints]
    if len(pts) >= 2 and pts[0] != pts[-1]:
        pts.append(pts[0])
    return pts


def _drop_closure(poly: List[Point]) -> List[Point]:
    if len(poly) > 1 and poly[0] == poly[-1]:
        return poly[:-1]
    return poly


def _ensure_closed(poly: List[Point]) -> List[Point]:
    if not poly:
        return poly
    if poly[0] != poly[-1]:
        return list(poly) + [poly[0]]
    return list(poly)


def _iter_polygons(geom) -> List["Polygon"]:
    if geom is None or geom.is_empty:
        return []
    if geom.geom_type == "Polygon":
        return [geom]
    if geom.geom_type == "MultiPolygon":
        return list(geom.geoms)
    if geom.geom_type == "GeometryCollection":
        out = []
        for g in geom.geoms:
            out.extend(_iter_polygons(g))
        return out
    return []


def _make_shapely_polygon(outer: List[Point], holes: List[List[Point]]):
    if not _HAS_SHAPELY:
        raise RuntimeError("Hybrid/contour planning requires shapely>=2.0 to be installed")
    outer = _drop_closure(list(outer))
    holes = [_drop_closure(list(h)) for h in holes]
    poly = Polygon(outer, holes)
    if not poly.is_valid:
        poly = poly.buffer(0)
    return poly


def _contour_passes(geom, reverse: bool, offset_index: int) -> List[CoveragePass]:
    passes: List[CoveragePass] = []
    for poly in _iter_polygons(geom):
        rings = [("outer", poly.exterior)] + [("hole", r) for r in poly.interiors]
        for ring_type, ring in rings:
            coords = list(ring.coords)
            coords = _drop_closure(coords)
            if len(coords) < 2:
                continue
            if reverse:
                coords = list(reversed(coords))
            points = [PathPoint(x=float(x), y=float(y), heading=None, is_coverage=True) for x, y in coords]
            passes.append(CoveragePass(
                points=points,
                metadata={"pattern": "contour", "ring": ring_type, "offset_index": offset_index},
            ))
    return passes


def _scanline_passes(outer: List[Point], holes: List[List[Point]], config: PlannerConfig) -> List[CoveragePass]:
    outer = _ensure_closed(outer)
    holes = [_ensure_closed(h) for h in holes]

    angle = config.sweep_angle_rad
    outer_r = _rotate_polygon(outer, -angle)
    holes_r = [_rotate_polygon(h, -angle) for h in holes]

    min_x, min_y, max_x, max_y = _bounds(outer_r)
    line_spacing = config.line_spacing_m
    if config.max_scanlines is not None:
        est_lines = int(math.ceil((max_y - min_y) / line_spacing)) + 1
        if est_lines > config.max_scanlines:
            raise ValueError(f"scanline count too high ({est_lines})")

    passes: List[CoveragePass] = []
    line_index = 0
    y = min_y
    while y <= max_y + config.y_epsilon:
        outer_segments = _polygon_scanline_segments(outer_r, y)
        if not outer_segments:
            y += line_spacing
            line_index += 1
            continue

        hole_segments = []
        for hp in holes_r:
            hole_segments.extend(_polygon_scanline_segments(hp, y))

        segments = _subtract_segments(outer_segments, hole_segments)
        if segments:
            is_left_to_right = (line_index % 2 == 0)
            passes.extend(
                _segments_to_passes(
                    segments,
                    y,
                    is_left_to_right,
                    angle,
                    line_index,
                    config.min_segment_length_m,
                )
            )
            if config.max_passes is not None and len(passes) > config.max_passes:
                raise ValueError(f"pass count too high ({len(passes)})")

        y += line_spacing
        line_index += 1

    return passes


def _rotate_point(p: Point, angle: float) -> Point:
    x, y = p
    c = math.cos(angle)
    s = math.sin(angle)
    return (x * c - y * s, x * s + y * c)


def _rotate_polygon(poly: List[Point], angle: float) -> List[Point]:
    return [_rotate_point(p, angle) for p in poly]


def _bounds(poly: List[Point]) -> Tuple[float, float, float, float]:
    xs = [p[0] for p in poly]
    ys = [p[1] for p in poly]
    return (min(xs), min(ys), max(xs), max(ys))


def _polygon_scanline_segments(poly: List[Point], y: float) -> List[Segment]:
    # Even-odd fill rule; skip horizontal edges.
    xs: List[float] = []
    n = len(poly)
    if n < 3:
        return []

    for i in range(n - 1):
        x1, y1 = poly[i]
        x2, y2 = poly[i + 1]
        if y1 == y2:
            continue
        y_min = min(y1, y2)
        y_max = max(y1, y2)
        if y >= y_min and y < y_max:
            t = (y - y1) / (y2 - y1)
            xs.append(x1 + t * (x2 - x1))

    if len(xs) < 2:
        return []

    xs.sort()
    segments = []
    for i in range(0, len(xs) - 1, 2):
        a = xs[i]
        b = xs[i + 1]
        if a != b:
            segments.append((min(a, b), max(a, b)))
    return segments


def _merge_segments(segments: Iterable[Segment]) -> List[Segment]:
    merged: List[Segment] = []
    for seg in sorted(segments, key=lambda s: s[0]):
        if not merged:
            merged.append(seg)
            continue
        prev = merged[-1]
        if seg[0] <= prev[1]:
            merged[-1] = (prev[0], max(prev[1], seg[1]))
        else:
            merged.append(seg)
    return merged


def _subtract_segments(outer: List[Segment], holes: List[Segment]) -> List[Segment]:
    if not outer:
        return []
    if not holes:
        return outer

    outer = _merge_segments(outer)
    holes = _merge_segments(holes)

    result: List[Segment] = []
    hi = 0
    for a, b in outer:
        start = a
        while hi < len(holes) and holes[hi][1] <= a:
            hi += 1
        hj = hi
        while hj < len(holes) and holes[hj][0] < b:
            h0, h1 = holes[hj]
            if h0 > start:
                result.append((start, min(h0, b)))
            start = max(start, h1)
            if start >= b:
                break
            hj += 1
        if start < b:
            result.append((start, b))
    return result


def _segments_to_passes(
    segments: List[Segment],
    y: float,
    left_to_right: bool,
    angle: float,
    line_index: int,
    min_len: float,
) -> List[CoveragePass]:
    if not segments:
        return []

    ordered = segments if left_to_right else list(reversed(segments))
    passes: List[CoveragePass] = []

    for seg in ordered:
        x0, x1 = seg
        if abs(x1 - x0) < min_len:
            continue
        if left_to_right:
            start = (x0, y)
            end = (x1, y)
        else:
            start = (x1, y)
            end = (x0, y)

        # Rotate back to world frame.
        p0 = _rotate_point(start, angle)
        p1 = _rotate_point(end, angle)

        heading = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        points = [
            PathPoint(x=p0[0], y=p0[1], heading=heading, is_coverage=True),
            PathPoint(x=p1[0], y=p1[1], heading=heading, is_coverage=True),
        ]
        metadata = {
            "line_index": line_index,
            "line_y": y,
            "direction": "ltr" if left_to_right else "rtl",
            "pattern": "scanline",
        }
        passes.append(CoveragePass(points=points, metadata=metadata))

    return passes
