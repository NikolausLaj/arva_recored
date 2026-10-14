"""Publish explicit operator timing markers into a recorded test bag."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any, Sequence

from .yaml_utils import ProtocolError, load_yaml


EVENT_TOPIC = '/arva_test_protocol/event'


def _load_metadata(run_directory: Path) -> tuple[Path, dict[str, Any]]:
    resolved_directory = run_directory.expanduser().resolve()
    metadata = load_yaml(resolved_directory / 'metadata.yaml')
    return resolved_directory, metadata


def publish_marker(
    run_directory: Path,
    event: str,
    confirmed: bool = False,
    repeat_count: int = 5,
    repeat_period_s: float = 0.2,
) -> int:
    """Publish one event after an explicit operator confirmation."""
    _, metadata = _load_metadata(run_directory)
    if not confirmed:
        print('[arva test protocol] waiting for operator: re-run with --confirm to publish marker')
        return 2
    if metadata.get('status') != 'recording':
        raise ProtocolError('event markers may only be published while the run is recording')
    if repeat_count < 1 or repeat_period_s <= 0.0:
        raise ProtocolError('repeat_count and repeat_period_s must be greater than zero')

    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as error:
        raise ProtocolError('event markers require rclpy and std_msgs.') from error

    rclpy.init()
    node = Node('arva_test_event_marker')
    publisher = node.create_publisher(String, EVENT_TOPIC, 10)
    message = String()
    message.data = json.dumps(
        {
            'event': event,
            'run_id': metadata.get('run_id'),
            'test_type': metadata.get('test_type'),
        },
        sort_keys=True,
    )
    for _ in range(repeat_count):
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=repeat_period_s)
    node.destroy_node()
    rclpy.shutdown()
    print(f'[arva test protocol] event marker published: {event} ({repeat_count} times)')
    return 0


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Publish an explicit operator event marker without commanding a vehicle.'
    )
    parser.add_argument('--run-dir', required=True, help='Recording run directory')
    parser.add_argument('--event', required=True, help='Short event name, such as terrain_step_1')
    parser.add_argument(
        '--confirm', action='store_true', help='Explicitly confirm marker publication'
    )
    parser.add_argument('--repeat-count', type=int, default=5)
    parser.add_argument('--repeat-period-s', type=float, default=0.2)
    return parser


def mark_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for the event marker."""
    parser = _parser()
    arguments_namespace = parser.parse_args(arguments)
    try:
        return publish_marker(
            Path(arguments_namespace.run_dir),
            arguments_namespace.event,
            arguments_namespace.confirm,
            arguments_namespace.repeat_count,
            arguments_namespace.repeat_period_s,
        )
    except ProtocolError as error:
        parser.error(str(error))
    return 1


if __name__ == '__main__':
    sys.exit(mark_main())