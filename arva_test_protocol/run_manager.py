"""Prepare, record, and close operator-supervised experiment runs."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import math
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
from typing import Any, Sequence

from .trajectory import parse_xy_waypoints
from .yaml_utils import ProtocolError, dump_yaml, load_yaml, merge_mappings


_IDENTIFIER_PATTERN = re.compile(r'^[A-Za-z0-9][A-Za-z0-9_-]*$')
_RUN_DIRECTORY_PATTERN = re.compile(r'^run_(\d+)$')
_RUN_STATES = {'prepared', 'recording', 'recording_stopped', 'completed', 'failed', 'aborted'}


def _print_status(message: str) -> None:
    print(f'[arva test protocol] {message}')


def _timestamp() -> str:
    return datetime.now(timezone.utc).astimezone().isoformat(timespec='seconds')


def _identifier(value: str, description: str) -> str:
    if not _IDENTIFIER_PATTERN.fullmatch(value):
        raise ProtocolError(
            f'{description} must use only letters, digits, underscores, or hyphens: {value!r}'
        )
    return value


def _absolute_path(path_value: str | Path, base_directory: Path) -> Path:
    path = Path(path_value).expanduser()
    return path if path.is_absolute() else base_directory / path


def _git_commit(workspace_root: Path) -> str | None:
    try:
        result = subprocess.run(
            ['git', '-C', str(workspace_root), 'rev-parse', 'HEAD'],
            check=True,
            capture_output=True,
            text=True,
        )
    except (FileNotFoundError, subprocess.CalledProcessError):
        return None
    return result.stdout.strip() or None


def _next_run_id(test_directory: Path) -> str:
    existing_numbers = []
    if test_directory.is_dir():
        for child in test_directory.iterdir():
            match = _RUN_DIRECTORY_PATTERN.fullmatch(child.name)
            if match and child.is_dir():
                existing_numbers.append(int(match.group(1)))
    return f'{max(existing_numbers, default=0) + 1:03d}'


def _normalise_run_id(run_id: str | None, test_directory: Path) -> str:
    if run_id is None:
        return _next_run_id(test_directory)
    value = run_id[4:] if run_id.startswith('run_') else run_id
    return _identifier(value, 'run ID')


def _require_mapping(value: Any, description: str) -> dict[str, Any]:
    if value is None:
        return {}
    if not isinstance(value, dict):
        raise ProtocolError(f'{description} must be a YAML mapping')
    return value


def _valid_xy_position(value: Any) -> bool:
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        return False
    north, east = value[0], value[1]
    return all(
        isinstance(coordinate, (int, float))
        and not isinstance(coordinate, bool)
        and math.isfinite(coordinate)
        for coordinate in (north, east)
    )


def _has_transmitter_ground_truth(config: dict[str, Any], metadata: dict[str, Any]) -> bool:
    metadata_position = metadata.get('transmitter_position')
    if isinstance(metadata_position, dict):
        metadata_position = metadata_position.get('meters')
    if _valid_xy_position(metadata_position):
        return True
    test = _require_mapping(config.get('test'), 'test')
    return _valid_xy_position(test.get('transmitter_position_ned_m'))


def _valid_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(value)
    )


def _validate_intervals(
    entries: Any, description: str, required_numeric_fields: tuple[str, ...] = ()
) -> None:
    if not isinstance(entries, list) or not entries:
        raise ProtocolError(f'{description} must be populated before recording')
    for index, raw_entry in enumerate(entries, start=1):
        entry = _require_mapping(raw_entry, f'{description}[{index}]')
        has_start = (
            _valid_number(entry.get('start_offset_s'))
            or isinstance(entry.get('start_event'), str)
        )
        has_end = (
            _valid_number(entry.get('end_offset_s'))
            or isinstance(entry.get('end_event'), str)
            or entry.get('end_of_bag') is True
        )
        if not has_start or not has_end:
            raise ProtocolError(
                f'{description}[{index}] must define start and end offsets, '
                'event markers, or end_of_bag for the end boundary'
            )
        for field in required_numeric_fields:
            if not _valid_number(entry.get(field)):
                raise ProtocolError(f'{description}[{index}].{field} must be numeric')


def validate_recording_configuration(
    config: dict[str, Any], metadata: dict[str, Any]
) -> None:
    """Reject templates missing inputs required for their configured analysis."""
    analysis = _require_mapping(config.get('analysis'), 'analysis')
    kind = analysis.get('kind')
    test = _require_mapping(config.get('test'), 'test')

    if kind == 'arva_characterization':
        _validate_intervals(
            analysis.get('samples'),
            'analysis.samples',
            ('expected_distance_m', 'expected_angle_rad'),
        )
    elif kind == 'heading_controller':
        if not (
            analysis.get('use_active_mode_interval') is True
            and analysis.get('segments') == []
        ):
            _validate_intervals(analysis.get('segments'), 'analysis.segments')
    elif kind == 'terrain_follow':
        _validate_intervals(
            analysis.get('segments'), 'analysis.segments', ('commanded_agl_m',)
        )
    elif kind == 'waypoint_follow':
        parse_xy_waypoints(
            test.get('planned_waypoints_ned_m'), 'test.planned_waypoints_ned_m'
        )
    elif kind in {'fluxline_follow', 'fine_search', 'full_search'}:
        if not _has_transmitter_ground_truth(config, metadata):
            raise ProtocolError(
                'a finite surveyed transmitter position must be set in test configuration '
                'or metadata before recording'
            )
        if kind == 'full_search':
            parse_xy_waypoints(
                test.get('planned_coverage_waypoints_ned_m'),
                'test.planned_coverage_waypoints_ned_m',
            )
    else:
        raise ProtocolError(f'unsupported analysis.kind for recording: {kind!r}')


def recording_topics(config: dict[str, Any]) -> list[str]:
    """Return validated, explicit ROS topics from a protocol configuration."""
    recording = _require_mapping(config.get('recording'), 'recording')
    topics = recording.get('topics')
    if not isinstance(topics, list) or not topics:
        raise ProtocolError('recording.topics must contain at least one topic')

    validated_topics = []
    for topic in topics:
        if not isinstance(topic, str) or not topic.startswith('/'):
            raise ProtocolError('each recording topic must be an absolute ROS topic name')
        validated_topics.append(topic)
    return validated_topics


def _append_state(metadata: dict[str, Any], state: str, details: str | None = None) -> None:
    if state not in _RUN_STATES:
        raise ProtocolError(f'unsupported run state: {state}')
    entry: dict[str, str] = {'timestamp': _timestamp(), 'state': state}
    if details:
        entry['details'] = details
    metadata['status'] = state
    metadata.setdefault('status_history', []).append(entry)


def _copy_parameter_files(
    config: dict[str, Any], workspace_root: Path, run_directory: Path
) -> list[str]:
    artifacts = _require_mapping(config.get('artifacts'), 'artifacts')
    parameter_files = artifacts.get('parameter_files', [])
    if not isinstance(parameter_files, list):
        raise ProtocolError('artifacts.parameter_files must be a list')

    copied_files = []
    destination_directory = run_directory / 'parameters'
    for parameter_file in parameter_files:
        if not isinstance(parameter_file, str):
            raise ProtocolError('each artifacts.parameter_files value must be a path string')
        source = _absolute_path(parameter_file, workspace_root)
        if not source.is_file():
            raise ProtocolError(f'parameter file does not exist: {source}')
        destination = destination_directory / source.name
        if destination.exists():
            raise ProtocolError(
                f'parameter files have duplicate filenames: {destination.name}'
            )
        destination_directory.mkdir(parents=True, exist_ok=True)
        shutil.copy2(source, destination)
        copied_files.append(str(destination.relative_to(run_directory)))
    return copied_files


def prepare_run(
    config_path: Path,
    output_root: Path,
    workspace_root: Path,
    run_id: str | None = None,
    operator: str | None = None,
    notes: str | None = None,
    metadata_path: Path | None = None,
    software_revision: str | None = None,
) -> Path:
    """Create an immutable experiment directory before the operator flies."""
    config = load_yaml(config_path)
    test_type = config.get('test_type')
    if not isinstance(test_type, str):
        raise ProtocolError('test_type must be a string')
    test_type = _identifier(test_type, 'test type')
    topics = recording_topics(config)

    test_directory = output_root / test_type
    current_run_id = _normalise_run_id(run_id, test_directory)
    run_directory = test_directory / f'run_{current_run_id}'
    if run_directory.exists():
        raise ProtocolError(f'run directory already exists: {run_directory}')

    metadata_defaults = _require_mapping(
        config.get('metadata_defaults'), 'metadata_defaults'
    )
    metadata_override = load_yaml(metadata_path) if metadata_path else {}
    metadata = merge_mappings(metadata_defaults, metadata_override)
    metadata.update(
        {
            'test_type': test_type,
            'run_id': current_run_id,
            'date': _timestamp(),
            'software_commit': (
                metadata.get('software_commit')
                or software_revision
                or _git_commit(workspace_root)
            ),
            'workspace_root': str(workspace_root),
            'configuration_source': str(config_path),
        }
    )
    if operator:
        metadata['operator'] = operator
    if notes:
        metadata['notes'] = notes
    _append_state(metadata, 'prepared')

    run_directory.mkdir(parents=True)
    shutil.copy2(config_path, run_directory / 'config.yaml')
    (run_directory / 'recording_topics.txt').write_text(
        '\n'.join(topics) + '\n', encoding='utf-8'
    )
    metadata['parameter_snapshots'] = _copy_parameter_files(
        config, workspace_root, run_directory
    )
    dump_yaml(run_directory / 'metadata.yaml', metadata)
    return run_directory


def _load_run_files(run_directory: Path) -> tuple[dict[str, Any], dict[str, Any], Path]:
    resolved_run_directory = run_directory.expanduser().resolve()
    config = load_yaml(resolved_run_directory / 'config.yaml')
    metadata = load_yaml(resolved_run_directory / 'metadata.yaml')
    return config, metadata, resolved_run_directory


def _write_metadata(run_directory: Path, metadata: dict[str, Any]) -> None:
    dump_yaml(run_directory / 'metadata.yaml', metadata)


def record_run(
    run_directory: Path,
    ros2_command: str = 'ros2',
    confirmed: bool = False,
    dry_run: bool = False,
    storage_id: str = 'sqlite3',
) -> int:
    """Record configured topics after a deliberate operator confirmation."""
    config, metadata, resolved_run_directory = _load_run_files(run_directory)
    test_type = metadata.get('test_type', 'unknown')
    topics = recording_topics(config)
    _print_status(f'test loaded: {test_type}')
    _print_status(f'configuration loaded: {resolved_run_directory / "config.yaml"}')

    if metadata.get('status') != 'prepared':
        raise ProtocolError(
            'only a prepared run can start recording; create a new run for another attempt'
        )
    if not confirmed:
        _print_status('waiting for operator: re-run with --confirm to start recording')
        return 2

    validate_recording_configuration(config, metadata)

    # PX4 publishes best-effort sensor data. Discovery may initially see no
    # publishers and choose reliable subscriptions, which then reject PX4.
    qos_overrides = {
        topic: {
            'reliability': 'best_effort',
            'durability': 'volatile',
            'history': 'keep_last',
            'depth': 10,
        }
        for topic in topics if topic.startswith('/fmu/out/')
    }
    qos_arguments = []
    if qos_overrides:
        qos_path = resolved_run_directory / 'qos_overrides.yaml'
        dump_yaml(qos_path, qos_overrides)
        qos_arguments = ['--qos-profile-overrides-path', str(qos_path)]

    bag_directory = resolved_run_directory / 'bag'
    command = [
        ros2_command,
        'bag',
        'record',
        '--storage',
        storage_id,
        '--include-unpublished-topics',
        '--output',
        str(bag_directory),
        *qos_arguments,
        *topics,
    ]
    if dry_run:
        _print_status(f'dry run command: {subprocess.list2cmdline(command)}')
        return 0

    _append_state(metadata, 'recording')
    metadata['recording_started_at'] = _timestamp()
    metadata['bag_path'] = str(bag_directory.relative_to(resolved_run_directory))
    metadata['storage_id'] = storage_id
    _write_metadata(resolved_run_directory, metadata)

    _print_status(f'recording started: {bag_directory}')
    _print_status('test active; use Ctrl-C to stop recording after the operator ends the test')
    try:
        process = subprocess.Popen(command)
    except FileNotFoundError as error:
        _append_state(metadata, 'failed', f'could not start {ros2_command}: {error}')
        _write_metadata(resolved_run_directory, metadata)
        raise ProtocolError(f'could not start {ros2_command}: {error}') from error

    interrupted_by_operator = False
    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        interrupted_by_operator = True
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
        return_code = process.wait()

    metadata = load_yaml(resolved_run_directory / 'metadata.yaml')
    metadata['recording_stopped_at'] = _timestamp()
    if return_code == 0 or interrupted_by_operator:
        _append_state(metadata, 'recording_stopped')
        _print_status('recording stopped; mark the flight outcome with arva_test_finish')
        result = 0
    else:
        _append_state(metadata, 'failed', f'ros2 bag record exited with code {return_code}')
        _print_status(f'test failed: ros2 bag record exited with code {return_code}')
        result = return_code
    _write_metadata(resolved_run_directory, metadata)
    return result


def finish_run(run_directory: Path, outcome: str, notes: str | None = None) -> None:
    """Store the operator-declared outcome after recording has stopped."""
    if outcome not in {'completed', 'failed', 'aborted'}:
        raise ProtocolError(f'unsupported outcome: {outcome}')

    _, metadata, resolved_run_directory = _load_run_files(run_directory)
    current_state = metadata.get('status')
    if current_state == 'prepared' and outcome == 'completed':
        raise ProtocolError('a prepared run cannot be marked completed without recording')
    if current_state not in {'prepared', 'recording_stopped', 'failed'}:
        raise ProtocolError(f'run cannot be finished from state: {current_state}')

    metadata['finished_at'] = _timestamp()
    metadata['outcome'] = outcome
    if notes:
        metadata['notes'] = notes
    _append_state(metadata, outcome)
    _write_metadata(resolved_run_directory, metadata)
    _print_status(f'test {outcome}: {resolved_run_directory}')


def _prepare_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Prepare a reproducible ARVA flight-test run without commanding a vehicle.'
    )
    parser.add_argument('--config', required=True, help='Test configuration YAML file')
    parser.add_argument(
        '--output-root', default='tests/real_world', help='Directory that will contain run folders'
    )
    parser.add_argument(
        '--workspace-root', default='.', help='Workspace root used for parameter-file snapshots'
    )
    parser.add_argument('--run-id', help='Optional run ID; defaults to the next three-digit ID')
    parser.add_argument('--operator', help='Operator name recorded in metadata.yaml')
    parser.add_argument(
        '--software-revision',
        help='Source revision/tag recorded when the workspace has no Git metadata',
    )
    parser.add_argument('--notes', help='Initial operator notes recorded in metadata.yaml')
    parser.add_argument(
        '--metadata-file', help='Run-specific metadata YAML merged over metadata_defaults'
    )
    return parser


def prepare_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for creating a new experiment run directory."""
    parser = _prepare_parser()
    arguments_namespace = parser.parse_args(arguments)
    workspace_root = Path(arguments_namespace.workspace_root).expanduser().resolve()
    config_path = _absolute_path(arguments_namespace.config, Path.cwd()).resolve()
    output_root = _absolute_path(arguments_namespace.output_root, workspace_root).resolve()
    metadata_path = (
        _absolute_path(arguments_namespace.metadata_file, Path.cwd()).resolve()
        if arguments_namespace.metadata_file
        else None
    )
    try:
        run_directory = prepare_run(
            config_path=config_path,
            output_root=output_root,
            workspace_root=workspace_root,
            run_id=arguments_namespace.run_id,
            operator=arguments_namespace.operator,
            notes=arguments_namespace.notes,
            metadata_path=metadata_path,
            software_revision=arguments_namespace.software_revision,
        )
    except ProtocolError as error:
        parser.error(str(error))
    _print_status(f'test loaded: {run_directory.parent.name}')
    _print_status(f'configuration loaded: {run_directory / "config.yaml"}')
    _print_status('waiting for operator; recording has not started')
    _print_status(f'prepared run: {run_directory}')
    return 0


def _record_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Start an explicitly confirmed rosbag recording for a prepared run.'
    )
    parser.add_argument('--run-dir', required=True, help='Prepared run directory')
    parser.add_argument(
        '--confirm', action='store_true', help='Explicitly confirm that recording should start'
    )
    parser.add_argument('--ros2-command', default='ros2', help='ROS 2 command executable')
    parser.add_argument(
        '--storage-id', default='sqlite3', choices=['sqlite3', 'mcap'],
        help='Bag storage format (MCAP requires rosbag2_storage_mcap)',
    )
    parser.add_argument(
        '--dry-run', action='store_true', help='Print the ros2 bag command without recording'
    )
    return parser


def record_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for deliberate rosbag recording."""
    parser = _record_parser()
    arguments_namespace = parser.parse_args(arguments)
    try:
        return record_run(
            run_directory=Path(arguments_namespace.run_dir),
            ros2_command=arguments_namespace.ros2_command,
            confirmed=arguments_namespace.confirm,
            dry_run=arguments_namespace.dry_run,
            storage_id=arguments_namespace.storage_id,
        )
    except ProtocolError as error:
        parser.error(str(error))
    return 1


def _finish_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Store the operator-declared result for a prepared flight-test run.'
    )
    parser.add_argument('--run-dir', required=True, help='Prepared run directory')
    parser.add_argument(
        '--outcome', required=True, choices=['completed', 'failed', 'aborted']
    )
    parser.add_argument('--notes', help='Final notes appended to metadata.yaml')
    return parser


def finish_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for closing an experiment run."""
    parser = _finish_parser()
    arguments_namespace = parser.parse_args(arguments)
    try:
        finish_run(
            run_directory=Path(arguments_namespace.run_dir),
            outcome=arguments_namespace.outcome,
            notes=arguments_namespace.notes,
        )
    except ProtocolError as error:
        parser.error(str(error))
    return 0


if __name__ == '__main__':
    sys.exit(prepare_main())
