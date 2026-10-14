import math

from arva_test_protocol.aggregate import metric_statistics, numeric_metrics
from arva_test_protocol.compare import compare_aggregates


def test_numeric_metrics_flattens_nested_numeric_results():
    metrics = numeric_metrics({'error': {'rmse': 0.5}, 'success': True})

    assert metrics == {'error.rmse': 0.5}


def test_metric_statistics_calculates_repeatability_values():
    summary = metric_statistics([1.0, 2.0, 3.0])

    assert summary['count'] == 3
    assert summary['median'] == 2.0
    assert math.isclose(summary['standard_deviation'], 1.0)


def test_compare_aggregates_uses_only_shared_recorded_metrics():
    comparison = compare_aggregates(
        {'test_type': 'fine_search', 'metric_statistics': {'localization_error': {'mean': 0.8}}},
        {'test_type': 'fine_search', 'metric_statistics': {'localization_error': {'mean': 0.5}}},
    )

    assert math.isclose(
        comparison['metrics']['localization_error']['difference_real_minus_simulation'], 0.3
    )