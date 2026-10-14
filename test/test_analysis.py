import math

from arva_test_protocol.analysis import error_metrics, normalize_angle


def test_normalize_angle_wraps_both_directions():
    assert math.isclose(normalize_angle(3.0 * math.pi), -math.pi)
    assert math.isclose(normalize_angle(-3.0 * math.pi), -math.pi)


def test_error_metrics_reports_signed_and_absolute_values():
    metrics = error_metrics([-2.0, 0.0, 2.0])

    assert metrics['count'] == 3
    assert metrics['mean_error'] == 0.0
    assert metrics['mean_absolute_error'] == 4.0 / 3.0
    assert metrics['maximum_absolute_error'] == 2.0
    assert math.isclose(metrics['root_mean_square_error'], math.sqrt(8.0 / 3.0))


def test_error_metrics_handles_empty_data():
    metrics = error_metrics([])

    assert metrics['count'] == 0
    assert metrics['root_mean_square_error'] is None