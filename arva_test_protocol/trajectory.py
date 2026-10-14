"""Small dependency-free helpers for PX4 local NED XY trajectories."""

from __future__ import annotations

import math
from typing import Iterable, Sequence

from .yaml_utils import ProtocolError


PointXY = tuple[float, float]


def xy_distance(first: PointXY, second: PointXY) -> float:
    """Return Euclidean distance in the horizontal NED plane."""
    return math.hypot(first[0] - second[0], first[1] - second[1])


def path_length(points: Iterable[PointXY]) -> float:
    """Return the accumulated horizontal length of a trajectory."""
    iterator = iter(points)
    try:
        previous = next(iterator)
    except StopIteration:
        return 0.0

    length = 0.0
    for point in iterator:
        length += xy_distance(previous, point)
        previous = point
    return length


def point_to_segment_distance(point: PointXY, start: PointXY, end: PointXY) -> float:
    """Return shortest XY distance from a point to a finite line segment."""
    segment_x = end[0] - start[0]
    segment_y = end[1] - start[1]
    segment_length_squared = segment_x * segment_x + segment_y * segment_y
    if segment_length_squared == 0.0:
        return xy_distance(point, start)

    projection = (
        (point[0] - start[0]) * segment_x + (point[1] - start[1]) * segment_y
    ) / segment_length_squared
    projection = min(1.0, max(0.0, projection))
    closest = (start[0] + projection * segment_x, start[1] + projection * segment_y)
    return xy_distance(point, closest)


def point_to_polyline_distance(point: PointXY, waypoints: Sequence[PointXY]) -> float:
    """Return shortest XY distance from a point to a waypoint polyline."""
    if not waypoints:
        raise ProtocolError('at least one waypoint is required')
    if len(waypoints) == 1:
        return xy_distance(point, waypoints[0])
    return min(
        point_to_segment_distance(point, start, end)
        for start, end in zip(waypoints, waypoints[1:])
    )


def parse_xy_waypoints(value: object, description: str) -> list[PointXY]:
    """Parse nested or flattened numeric NED waypoint pairs from YAML data."""
    if not isinstance(value, list) or not value:
        raise ProtocolError(f'{description} must contain at least two waypoint pairs')

    if all(isinstance(item, (int, float)) and not isinstance(item, bool) for item in value):
        if len(value) < 4 or len(value) % 2 != 0:
            raise ProtocolError(f'{description} must contain an even number of values')
        points = [(float(value[index]), float(value[index + 1])) for index in range(0, len(value), 2)]
    else:
        points = []
        for index, item in enumerate(value, start=1):
            if not isinstance(item, (list, tuple)) or len(item) < 2:
                raise ProtocolError(f'{description}[{index}] must contain north and east values')
            north, east = item[0], item[1]
            if (
                isinstance(north, bool)
                or isinstance(east, bool)
                or not isinstance(north, (int, float))
                or not isinstance(east, (int, float))
            ):
                raise ProtocolError(f'{description}[{index}] values must be numeric')
            points.append((float(north), float(east)))

    if len(points) < 2:
        raise ProtocolError(f'{description} must contain at least two waypoint pairs')
    if not all(math.isfinite(coordinate) for point in points for coordinate in point):
        raise ProtocolError(f'{description} values must be finite')
    return points