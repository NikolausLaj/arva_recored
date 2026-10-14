"""Aggregate repeated ARVA test-run summaries without changing flight data."""

from __future__ import annotations

import argparse
import csv
from pathlib import Path
from statistics import mean, median, stdev
import sys
from typing import Any, Iterable, Sequence

from .yaml_utils import ProtocolError, dump_yaml, load_yaml


def numeric_metrics(value: Any, prefix: str = '') -> dict[str, float]:
    """Flatten finite numeric leaves from a nested result mapping."""
    metrics = {}
    if isinstance(value, dict):
        for key, nested_value in value.items():
            nested_prefix = f'{prefix}.{key}' if prefix else str(key)
            metrics.update(numeric_metrics(nested_value, nested_prefix))
    elif (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and float(value) == float(value)
    ):
        metrics[prefix] = float(value)
    return metrics


def metric_statistics(values: Iterable[float]) -> dict[str, float | int | None]:
    """Calculate reusable statistics for a collection of run-level values."""
    samples = list(values)
    if not samples:
        return {
            'count': 0,
            'mean': None,
            'median': None,
            'maximum': None,
            'standard_deviation': None,
        }
    return {
        'count': len(samples),
        'mean': mean(samples),
        'median': median(samples),
        'maximum': max(samples),
        'standard_deviation': stdev(samples) if len(samples) > 1 else 0.0,
    }


def aggregate_test_directory(test_directory: Path) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    """Read all completed run summaries beneath a single test-type directory."""
    resolved_directory = test_directory.expanduser().resolve()
    if not resolved_directory.is_dir():
        raise ProtocolError(f'test directory does not exist: {resolved_directory}')

    run_rows = []
    metric_values: dict[str, list[float]] = {}
    success_values = []
    test_type = resolved_directory.name
    for run_directory in sorted(resolved_directory.glob('run_*')):
        summary_path = run_directory / 'results' / 'summary.yaml'
        metadata_path = run_directory / 'metadata.yaml'
        if not summary_path.is_file() or not metadata_path.is_file():
            continue
        summary = load_yaml(summary_path)
        metadata = load_yaml(metadata_path)
        summary_metrics = summary.get('metrics')
        if not isinstance(summary_metrics, dict):
            continue
        flattened = numeric_metrics(summary_metrics)
        success = summary_metrics.get('success')
        if isinstance(success, bool):
            success_values.append(success)
        run_row = {
            'run_id': metadata.get('run_id', run_directory.name),
            'operator_outcome': metadata.get('outcome'),
            'analysis_kind': summary.get('analysis_kind'),
            'success': success if isinstance(success, bool) else None,
            **flattened,
        }
        run_rows.append(run_row)
        for metric_name, metric_value in flattened.items():
            metric_values.setdefault(metric_name, []).append(metric_value)

    summary = {
        'test_type': test_type,
        'run_count': len(run_rows),
        'success': {
            'count': len(success_values),
            'successful_runs': sum(success_values),
            'success_rate': sum(success_values) / len(success_values) if success_values else None,
        },
        'metric_statistics': {
            metric_name: metric_statistics(values)
            for metric_name, values in sorted(metric_values.items())
        },
    }
    return summary, run_rows


def _write_runs_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    fields = sorted({key for row in rows for key in row})
    with path.open('w', newline='', encoding='utf-8') as file_handle:
        writer = csv.DictWriter(file_handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Aggregate repeated, post-flight ARVA test-run results.'
    )
    parser.add_argument('--test-dir', required=True, help='Directory containing run_<id> folders')
    parser.add_argument('--output-dir', help='Default: <test-dir>/aggregate')
    return parser


def aggregate_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for test-run aggregation."""
    parser = _parser()
    arguments_namespace = parser.parse_args(arguments)
    try:
        test_directory = Path(arguments_namespace.test_dir)
        summary, rows = aggregate_test_directory(test_directory)
        output_directory = (
            Path(arguments_namespace.output_dir).expanduser().resolve()
            if arguments_namespace.output_dir
            else test_directory.expanduser().resolve() / 'aggregate'
        )
        output_directory.mkdir(parents=True, exist_ok=True)
        dump_yaml(output_directory / 'summary.yaml', summary)
        _write_runs_csv(output_directory / 'runs.csv', rows)
    except ProtocolError as error:
        parser.error(str(error))
    print(f'[arva test protocol] aggregation completed: {output_directory}')
    return 0


if __name__ == '__main__':
    sys.exit(aggregate_main())