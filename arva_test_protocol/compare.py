"""Compare two aggregate result summaries, typically simulation and real data."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Any, Sequence

from .yaml_utils import ProtocolError, dump_yaml, load_yaml


def compare_aggregates(
    real_summary: dict[str, Any], simulation_summary: dict[str, Any]
) -> dict[str, Any]:
    """Create a comparison only for numeric metrics present in both summaries."""
    real_metrics = real_summary.get('metric_statistics')
    simulation_metrics = simulation_summary.get('metric_statistics')
    if not isinstance(real_metrics, dict) or not isinstance(simulation_metrics, dict):
        raise ProtocolError('both inputs must be aggregate summaries with metric_statistics')

    comparisons = {}
    for metric_name in sorted(set(real_metrics).intersection(simulation_metrics)):
        real_mean = real_metrics[metric_name].get('mean')
        simulation_mean = simulation_metrics[metric_name].get('mean')
        if not isinstance(real_mean, (int, float)) or not isinstance(simulation_mean, (int, float)):
            continue
        comparisons[metric_name] = {
            'simulation_mean': simulation_mean,
            'real_mean': real_mean,
            'difference_real_minus_simulation': real_mean - simulation_mean,
            'real_to_simulation_ratio': (
                real_mean / simulation_mean if simulation_mean != 0.0 else None
            ),
        }
    return {
        'simulation_test_type': simulation_summary.get('test_type'),
        'real_test_type': real_summary.get('test_type'),
        'metrics': comparisons,
    }


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Compare recorded simulation and real-world aggregate summaries.'
    )
    parser.add_argument('--simulation-summary', required=True)
    parser.add_argument('--real-summary', required=True)
    parser.add_argument('--output', required=True)
    return parser


def compare_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for simulation-to-real summary comparison."""
    parser = _parser()
    arguments_namespace = parser.parse_args(arguments)
    try:
        comparison = compare_aggregates(
            load_yaml(Path(arguments_namespace.real_summary)),
            load_yaml(Path(arguments_namespace.simulation_summary)),
        )
        output_path = Path(arguments_namespace.output).expanduser().resolve()
        dump_yaml(output_path, comparison)
    except ProtocolError as error:
        parser.error(str(error))
    print(f'[arva test protocol] comparison completed: {output_path}')
    return 0


if __name__ == '__main__':
    sys.exit(compare_main())