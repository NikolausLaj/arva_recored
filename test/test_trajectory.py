import math

from arva_test_protocol.trajectory import (
    parse_xy_waypoints,
    path_length,
    point_to_polyline_distance,
    point_to_segment_distance,
)


def test_path_length_accumulates_xy_segments():
    assert path_length([(0.0, 0.0), (3.0, 4.0), (3.0, 8.0)]) == 9.0


def test_point_to_segment_distance_clamps_to_endpoint():
    assert point_to_segment_distance((4.0, 3.0), (0.0, 0.0), (2.0, 0.0)) == math.sqrt(13.0)


def test_point_to_polyline_distance_uses_nearest_leg():
    distance = point_to_polyline_distance((1.0, 2.0), [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0)])
    assert distance == 2.0


def test_parse_xy_waypoints_accepts_nested_and_flat_values():
    assert parse_xy_waypoints([[0.0, 1.0], [2.0, 3.0]], 'waypoints') == [(0.0, 1.0), (2.0, 3.0)]
    assert parse_xy_waypoints([0.0, 1.0, 2.0, 3.0], 'waypoints') == [(0.0, 1.0), (2.0, 3.0)]