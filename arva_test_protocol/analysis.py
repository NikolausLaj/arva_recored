"""Offline metrics and plots for recorded ARVA real-world test runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from statistics import mean, stdev
import sys
from typing import Any, Callable, Iterable, Sequence

from .bag_reader import BagMessage, read_messages
from .trajectory import (
    parse_xy_waypoints,
    path_length,
    point_to_polyline_distance,
    xy_distance,
)
from .yaml_utils import ProtocolError, dump_yaml, load_yaml


PIEPS_TOPIC = '/pieps/measurement'
LOCAL_POSITION_TOPIC = '/fmu/out/vehicle_local_position'
TERRAIN_COMMAND_TOPIC = '/terrain_controller/cmd_vel'
MODE_STATE_TOPIC = '/arva_master/mode_state'
FLUX_COMMAND_TOPIC = '/fluxline_controller/cmd_vel'
FINE_STATE_TOPIC = '/finesearch_controller/state'
FINE_ESTIMATE_TOPIC = '/finesearch_controller/estimated_position'
FINE_COMPLETION_TOPIC = '/finesearch_controller/completed'
EVENT_TOPIC = '/arva_test_protocol/event'


def normalize_angle(angle_rad: float) -> float:
    """Normalize an angle to the half-open interval [-pi, pi)."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def error_metrics(errors: Iterable[float]) -> dict[str, float | int | None]:
    """Calculate reusable signed-error metrics from an iterable of samples."""
    values = list(errors)
    if not values:
        return {
            'count': 0,
            'mean_error': None,
            'mean_absolute_error': None,
            'maximum_absolute_error': None,
            'root_mean_square_error': None,
            'standard_deviation': None,
        }
    return {
        'count': len(values),
        'mean_error': mean(values),
        'mean_absolute_error': mean(abs(value) for value in values),
        'maximum_absolute_error': max(abs(value) for value in values),
        'root_mean_square_error': math.sqrt(mean(value * value for value in values)),
        'standard_deviation': stdev(values) if len(values) > 1 else 0.0,
    }


def _numeric(value: Any, description: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolError(f'{description} must be numeric')
    return float(value)


def _mapping(value: Any, description: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ProtocolError(f'{description} must be a mapping')
    return value


def _message_time_s(message: BagMessage, start_timestamp_ns: int) -> float:
    return (message.timestamp_ns - start_timestamp_ns) / 1_000_000_000.0


def _event_offsets(
    event_messages: list[BagMessage], start_timestamp_ns: int
) -> dict[str, list[float]]:
    """Map repeated JSON event markers to their first and later bag-time offsets."""
    offsets: dict[str, list[float]] = {}
    for message in sorted(event_messages, key=lambda item: item.timestamp_ns):
        try:
            payload = json.loads(message.message.data)
        except (AttributeError, TypeError, json.JSONDecodeError):
            continue
        event = payload.get('event') if isinstance(payload, dict) else None
        if isinstance(event, str):
            offsets.setdefault(event, []).append(
                _message_time_s(message, start_timestamp_ns)
            )
    return offsets


def _interval_bound(
    item: dict[str, Any],
    offset_key: str,
    event_key: str,
    event_offsets: dict[str, list[float]],
    description: str,
    end_of_bag_s: float | None = None,
) -> float:
    """Resolve an interval boundary from either a numeric offset or event name."""
    offset = item.get(offset_key)
    event = item.get(event_key)
    if offset is not None and event is not None:
        raise ProtocolError(f'{description} may define either {offset_key} or {event_key}, not both')
    if offset is not None:
        return _numeric(offset, f'{description}.{offset_key}')
    if isinstance(event, str):
        timestamps = event_offsets.get(event)
        if not timestamps:
            raise ProtocolError(f'{description} references missing event marker: {event}')
        return timestamps[0]
    if end_of_bag_s is not None and item.get('end_of_bag') is True:
        return end_of_bag_s
    raise ProtocolError(f'{description} must define {offset_key} or {event_key}')


def _uses_event_markers(analysis_config: dict[str, Any], collection_key: str) -> bool:
    collection = analysis_config.get(collection_key)
    return isinstance(collection, list) and any(
        isinstance(item, dict)
        and (item.get('start_event') is not None or item.get('end_event') is not None)
        for item in collection
    )


def _write_csv(path: Path, fieldnames: list[str], rows: Iterable[dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open('w', newline='', encoding='utf-8') as file_handle:
        writer = csv.DictWriter(file_handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def _try_plot(plot_function: Callable[[], None]) -> str | None:
    try:
        plot_function()
    except ImportError:
        return 'Matplotlib is unavailable; CSV and YAML results were still written.'
    return None


def _arva_rows(
    pieps_messages: list[BagMessage], analysis_config: dict[str, Any],
    event_messages: list[BagMessage] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    samples = analysis_config.get('samples')
    if not isinstance(samples, list) or not samples:
        raise ProtocolError(
            'analysis.samples must define timed, surveyed ARVA measurement samples'
        )
    if not pieps_messages:
        raise ProtocolError(f'bag contains no {PIEPS_TOPIC} messages')

    event_messages = event_messages or []
    start_timestamp_ns = min(
        pieps_messages[0].timestamp_ns,
        *[message.timestamp_ns for message in event_messages],
    )
    end_of_bag_s = (
        max(
            pieps_messages[-1].timestamp_ns,
            *[message.timestamp_ns for message in event_messages],
        )
        - start_timestamp_ns
    ) / 1_000_000_000.0
    event_offsets = _event_offsets(event_messages, start_timestamp_ns)
    rows = []
    sample_summaries = []
    for index, raw_sample in enumerate(samples, start=1):
        sample = _mapping(raw_sample, f'analysis.samples[{index}]')
        sample_id = str(sample.get('id', f'sample_{index:02d}'))
        start_offset_s = _interval_bound(
            sample,
            'start_offset_s',
            'start_event',
            event_offsets,
            f'analysis.samples[{index}]',
        )
        end_offset_s = _interval_bound(
            sample,
            'end_offset_s',
            'end_event',
            event_offsets,
            f'analysis.samples[{index}]',
            end_of_bag_s,
        )
        if end_offset_s <= start_offset_s:
            raise ProtocolError(f'analysis.samples[{index}] must have a positive time range')

        expected_distance = _numeric(
            sample.get('expected_distance_m'),
            f'analysis.samples[{index}].expected_distance_m',
        )
        expected_angle_value = sample.get('expected_angle_rad')
        expected_angle = (
            _numeric(expected_angle_value, f'analysis.samples[{index}].expected_angle_rad')
            if expected_angle_value is not None
            else None
        )
        sample_rows = []
        for bag_message in pieps_messages:
            time_s = _message_time_s(bag_message, start_timestamp_ns)
            if not start_offset_s <= time_s <= end_offset_s:
                continue
            measurement = bag_message.message
            distance_valid = bool(measurement.distance_valid)
            angle_valid = bool(measurement.angle_valid)
            distance = float(measurement.distance)
            angle = float(measurement.angle)
            row = {
                'sample_id': sample_id,
                'timestamp_ns': bag_message.timestamp_ns,
                'time_s': time_s,
                'expected_distance_m': expected_distance,
                'distance_m': distance,
                'distance_valid': distance_valid,
                'distance_error_m': distance - expected_distance if distance_valid else None,
                'expected_angle_rad': expected_angle,
                'angle_rad': angle,
                'angle_valid': angle_valid,
                'angle_error_rad': (
                    normalize_angle(angle - expected_angle)
                    if angle_valid and expected_angle is not None
                    else None
                ),
            }
            rows.append(row)
            sample_rows.append(row)

        distance_errors = [
            row['distance_error_m']
            for row in sample_rows
            if row['distance_error_m'] is not None
        ]
        angle_errors = [
            row['angle_error_rad']
            for row in sample_rows
            if row['angle_error_rad'] is not None
        ]
        total = len(sample_rows)
        sample_summaries.append(
            {
                'id': sample_id,
                'measurement_count': total,
                'distance_valid_rate': (
                    sum(row['distance_valid'] for row in sample_rows) / total if total else None
                ),
                'angle_valid_rate': (
                    sum(row['angle_valid'] for row in sample_rows) / total if total else None
                ),
                'valid_measurement_rate': (
                    sum(
                        row['distance_valid'] and row['angle_valid']
                        for row in sample_rows
                    )
                    / total
                    if total
                    else None
                ),
                'distance_error_m': error_metrics(distance_errors),
                'angle_error_rad': error_metrics(angle_errors),
            }
        )

    total_rows = len(rows)
    summary = {
        'measurement_count': total_rows,
        'distance_valid_rate': (
            sum(row['distance_valid'] for row in rows) / total_rows if total_rows else None
        ),
        'angle_valid_rate': (
            sum(row['angle_valid'] for row in rows) / total_rows if total_rows else None
        ),
        'valid_measurement_rate': (
            sum(row['distance_valid'] and row['angle_valid'] for row in rows) / total_rows
            if total_rows
            else None
        ),
        'distance_error_m': error_metrics(
            row['distance_error_m'] for row in rows if row['distance_error_m'] is not None
        ),
        'angle_error_rad': error_metrics(
            row['angle_error_rad'] for row in rows if row['angle_error_rad'] is not None
        ),
        'samples': sample_summaries,
    }
    return rows, summary


def _plot_arva(rows: list[dict[str, Any]], results_directory: Path) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    if not rows:
        return
    figure, axes = pyplot.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot([row['time_s'] for row in rows], [row['distance_m'] for row in rows])
    axes[0].set_ylabel('ARVA distance [m]')
    axes[0].grid(True)
    angle_rows = [row for row in rows if row['angle_error_rad'] is not None]
    axes[1].plot(
        [row['time_s'] for row in angle_rows],
        [math.degrees(row['angle_error_rad']) for row in angle_rows],
    )
    axes[1].set_xlabel('Time from bag start [s]')
    axes[1].set_ylabel('Angle error [deg]')
    axes[1].grid(True)
    figure.tight_layout()
    figure.savefig(results_directory / 'arva_measurements.png', dpi=200)
    pyplot.close(figure)


def _latest_commands(
    position_messages: list[BagMessage], command_messages: list[BagMessage]
) -> dict[int, float | None]:
    command_index = 0
    latest_command = None
    values = {}
    for position_message in position_messages:
        while (
            command_index < len(command_messages)
            and command_messages[command_index].timestamp_ns <= position_message.timestamp_ns
        ):
            latest_command = float(command_messages[command_index].message.linear.z)
            command_index += 1
        values[position_message.timestamp_ns] = latest_command
    return values


def _segment_metrics(
    rows: list[dict[str, Any]], segment: dict[str, Any], default_band_m: float,
    steady_state_window_s: float, segment_start_time_s: float,
) -> dict[str, Any]:
    error_values = [row['agl_error_m'] for row in rows]
    metrics: dict[str, Any] = {
        'sample_count': len(rows),
        'tracking_error_m': error_metrics(error_values),
    }
    if not rows:
        metrics.update(
            {
                'steady_state_error_m': None,
                'rise_time_s': None,
                'settling_time_s': None,
                'overshoot_m': None,
            }
        )
        return metrics

    end_time_s = rows[-1]['time_s']
    steady_state_rows = [
        row for row in rows if row['time_s'] >= end_time_s - steady_state_window_s
    ]
    metrics['steady_state_error_m'] = mean(
        row['agl_error_m'] for row in steady_state_rows
    )

    initial_value = segment.get('initial_agl_m')
    if initial_value is None:
        metrics.update(
            {
                'rise_time_s': None,
                'settling_time_s': None,
                'overshoot_m': None,
            }
        )
        return metrics

    initial_agl_m = _numeric(initial_value, 'segment.initial_agl_m')
    target_agl_m = rows[0]['commanded_agl_m']
    change = target_agl_m - initial_agl_m
    if abs(change) < 1e-9:
        metrics.update(
            {
                'rise_time_s': None,
                'settling_time_s': None,
                'overshoot_m': 0.0,
            }
        )
        return metrics

    direction = 1.0 if change > 0.0 else -1.0
    magnitude = abs(change)
    threshold_10 = initial_agl_m + 0.1 * change
    threshold_90 = initial_agl_m + 0.9 * change
    time_at_10 = next(
        (
            row['time_s']
            for row in rows
            if direction * (row['measured_agl_m'] - threshold_10) >= 0.0
        ),
        None,
    )
    time_at_90 = next(
        (
            row['time_s']
            for row in rows
            if direction * (row['measured_agl_m'] - threshold_90) >= 0.0
        ),
        None,
    )
    metrics['rise_time_s'] = (
        time_at_90 - time_at_10
        if time_at_10 is not None and time_at_90 is not None
        else None
    )
    settling_band_m = _numeric(segment.get('settling_band_m', default_band_m), 'segment.settling_band_m')
    metrics['settling_time_s'] = next(
        (
            row['time_s'] - segment_start_time_s
            for index, row in enumerate(rows)
            if all(abs(candidate['agl_error_m']) <= settling_band_m for candidate in rows[index:])
        ),
        None,
    )
    metrics['overshoot_m'] = max(
        0.0,
        max(direction * (row['measured_agl_m'] - target_agl_m) for row in rows),
    )
    metrics['step_magnitude_m'] = magnitude
    return metrics


def _terrain_rows(
    position_messages: list[BagMessage],
    command_messages: list[BagMessage],
    analysis_config: dict[str, Any],
    event_messages: list[BagMessage] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    segments = analysis_config.get('segments')
    if not isinstance(segments, list) or not segments:
        raise ProtocolError(
            'analysis.segments must define timed terrain step or tracking intervals'
        )
    if not position_messages:
        raise ProtocolError(f'bag contains no {LOCAL_POSITION_TOPIC} messages')

    position_messages = sorted(position_messages, key=lambda item: item.timestamp_ns)
    command_messages = sorted(command_messages, key=lambda item: item.timestamp_ns)
    event_messages = event_messages or []
    start_timestamp_ns = min(
        [
            position_messages[0].timestamp_ns,
            *[item.timestamp_ns for item in command_messages],
            *[item.timestamp_ns for item in event_messages],
        ]
    )
    end_of_bag_s = (
        max(
            position_messages[-1].timestamp_ns,
            *[item.timestamp_ns for item in command_messages],
            *[item.timestamp_ns for item in event_messages],
        )
        - start_timestamp_ns
    ) / 1_000_000_000.0
    event_offsets = _event_offsets(event_messages, start_timestamp_ns)
    commands = _latest_commands(position_messages, command_messages)
    default_band_m = _numeric(analysis_config.get('settling_band_m', 0.15), 'analysis.settling_band_m')
    steady_state_window_s = _numeric(
        analysis_config.get('steady_state_window_s', 5.0),
        'analysis.steady_state_window_s',
    )

    rows = []
    summaries = []
    total_range_samples = 0
    valid_range_samples = 0
    for index, raw_segment in enumerate(segments, start=1):
        segment = _mapping(raw_segment, f'analysis.segments[{index}]')
        segment_id = str(segment.get('id', f'segment_{index:02d}'))
        start_offset_s = _interval_bound(
            segment,
            'start_offset_s',
            'start_event',
            event_offsets,
            f'analysis.segments[{index}]',
        )
        end_offset_s = _interval_bound(
            segment,
            'end_offset_s',
            'end_event',
            event_offsets,
            f'analysis.segments[{index}]',
            end_of_bag_s,
        )
        target_agl_m = _numeric(
            segment.get('commanded_agl_m'), f'analysis.segments[{index}].commanded_agl_m'
        )
        if end_offset_s <= start_offset_s:
            raise ProtocolError(f'analysis.segments[{index}] must have a positive time range')

        segment_rows = []
        segment_range_samples = 0
        segment_valid_range_samples = 0
        for position_message in position_messages:
            time_s = _message_time_s(position_message, start_timestamp_ns)
            if not start_offset_s <= time_s <= end_offset_s:
                continue
            segment_range_samples += 1
            total_range_samples += 1
            position = position_message.message
            range_valid = bool(getattr(position, 'dist_bottom_valid', True))
            measured_agl_m = float(position.dist_bottom)
            if not range_valid or not math.isfinite(measured_agl_m):
                continue
            segment_valid_range_samples += 1
            valid_range_samples += 1
            row = {
                'segment_id': segment_id,
                'timestamp_ns': position_message.timestamp_ns,
                'time_s': time_s,
                'commanded_agl_m': target_agl_m,
                'measured_agl_m': measured_agl_m,
                'agl_error_m': measured_agl_m - target_agl_m,
                'north_m': getattr(position, 'x', None),
                'east_m': getattr(position, 'y', None),
                'down_m': getattr(position, 'z', None),
                'measured_vertical_velocity_ned_mps': getattr(position, 'vz', None),
                'commanded_vertical_velocity_ned_mps': commands[position_message.timestamp_ns],
            }
            rows.append(row)
            segment_rows.append(row)

        segment_summary = _segment_metrics(
            segment_rows,
            segment,
            default_band_m,
            steady_state_window_s,
            start_offset_s,
        )
        summaries.append(
            {
                'id': segment_id,
                'range_sample_count': segment_range_samples,
                'valid_range_sample_count': segment_valid_range_samples,
                'range_valid_rate': (
                    segment_valid_range_samples / segment_range_samples
                    if segment_range_samples
                    else None
                ),
                **segment_summary,
            }
        )

    summary = {
        'sample_count': len(rows),
        'range_sample_count': total_range_samples,
        'valid_range_sample_count': valid_range_samples,
        'range_valid_rate': (
            valid_range_samples / total_range_samples if total_range_samples else None
        ),
        'tracking_error_m': error_metrics(row['agl_error_m'] for row in rows),
        'segments': summaries,
    }
    return rows, summary


def _plot_terrain(rows: list[dict[str, Any]], results_directory: Path) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    if not rows:
        return
    figure, axes = pyplot.subplots(2, 1, figsize=(10, 7), sharex=True)
    axes[0].plot([row['time_s'] for row in rows], [row['measured_agl_m'] for row in rows], label='Measured AGL')
    axes[0].step(
        [row['time_s'] for row in rows],
        [row['commanded_agl_m'] for row in rows],
        where='post',
        label='Commanded AGL',
    )
    axes[0].set_ylabel('AGL [m]')
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot([row['time_s'] for row in rows], [row['agl_error_m'] for row in rows])
    axes[1].set_xlabel('Time from bag start [s]')
    axes[1].set_ylabel('AGL error [m]')
    axes[1].grid(True)
    figure.tight_layout()
    figure.savefig(results_directory / 'terrain_following.png', dpi=200)
    pyplot.close(figure)


def _mode_interval(
    state_messages: list[BagMessage],
    target_state: str,
    default_start_timestamp_ns: int,
    default_end_timestamp_ns: int,
) -> tuple[int, int]:
    """Return the first recorded interval for a Mode Executor state."""
    if not state_messages:
        raise ProtocolError(f'bag contains no {MODE_STATE_TOPIC} messages')

    transitions = []
    previous_state = None
    for message in sorted(state_messages, key=lambda item: item.timestamp_ns):
        state = str(message.message.data)
        if state != previous_state:
            transitions.append((message.timestamp_ns, state))
            previous_state = state

    for index, (start_timestamp_ns, state) in enumerate(transitions):
        if state != target_state:
            continue
        end_timestamp_ns = (
            transitions[index + 1][0]
            if index + 1 < len(transitions)
            else default_end_timestamp_ns
        )
        return max(start_timestamp_ns, default_start_timestamp_ns), end_timestamp_ns
    raise ProtocolError(f'bag does not contain Mode Executor state {target_state!r}')


def _state_transitions(state_messages: list[BagMessage]) -> list[tuple[int, str]]:
    """Return timestamped Mode Executor state changes, removing repeated values."""
    transitions = []
    previous_state = None
    for message in sorted(state_messages, key=lambda item: item.timestamp_ns):
        state = str(message.message.data)
        if state != previous_state:
            transitions.append((message.timestamp_ns, state))
            previous_state = state
    return transitions


def _latest_values(
    reference_messages: list[BagMessage],
    value_messages: list[BagMessage],
    extractor: Callable[[BagMessage], Any],
) -> dict[int, Any]:
    """Associate each reference message with the latest preceding value message."""
    value_index = 0
    current_value = None
    values = {}
    ordered_values = sorted(value_messages, key=lambda item: item.timestamp_ns)
    for reference in sorted(reference_messages, key=lambda item: item.timestamp_ns):
        while (
            value_index < len(ordered_values)
            and ordered_values[value_index].timestamp_ns <= reference.timestamp_ns
        ):
            current_value = extractor(ordered_values[value_index])
            value_index += 1
        values[reference.timestamp_ns] = current_value
    return values


def _response_metrics(
    rows: list[dict[str, Any]], error_key: str, tolerance: float,
    steady_state_window_s: float,
) -> dict[str, Any]:
    """Calculate settling, overshoot, oscillation, and steady-state metrics."""
    valid_rows = [row for row in rows if row[error_key] is not None]
    metrics = {'error': error_metrics(row[error_key] for row in valid_rows)}
    if not valid_rows:
        metrics.update(
            {
                'settling_time_s': None,
                'overshoot': None,
                'oscillation_crossings': 0,
                'steady_state_error': None,
            }
        )
        return metrics

    start_time_s = valid_rows[0]['time_s']
    metrics['settling_time_s'] = next(
        (
            row['time_s'] - start_time_s
            for index, row in enumerate(valid_rows)
            if all(abs(candidate[error_key]) <= tolerance for candidate in valid_rows[index:])
        ),
        None,
    )
    initial_error = valid_rows[0][error_key]
    initial_sign = 1.0 if initial_error >= 0.0 else -1.0
    opposite_errors = [
        -initial_sign * row[error_key]
        for row in valid_rows
        if -initial_sign * row[error_key] > 0.0
    ]
    metrics['overshoot'] = max(opposite_errors, default=0.0)

    significant_signs = [
        1 if row[error_key] > tolerance else -1
        for row in valid_rows
        if abs(row[error_key]) > tolerance
    ]
    metrics['oscillation_crossings'] = sum(
        previous != current
        for previous, current in zip(significant_signs, significant_signs[1:])
    )
    final_time_s = valid_rows[-1]['time_s']
    steady_state_rows = [
        row for row in valid_rows
        if row['time_s'] >= final_time_s - steady_state_window_s
    ]
    metrics['steady_state_error'] = mean(
        row[error_key] for row in steady_state_rows
    )
    return metrics


def _heading_rows(
    pieps_messages: list[BagMessage],
    position_messages: list[BagMessage],
    command_messages: list[BagMessage],
    state_messages: list[BagMessage],
    config: dict[str, Any],
    event_messages: list[BagMessage] | None = None,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not pieps_messages:
        raise ProtocolError(f'bag contains no {PIEPS_TOPIC} messages')
    if not position_messages:
        raise ProtocolError(f'bag contains no {LOCAL_POSITION_TOPIC} messages')
    pieps_messages = sorted(pieps_messages, key=lambda item: item.timestamp_ns)
    position_messages = sorted(position_messages, key=lambda item: item.timestamp_ns)
    start_timestamp_ns, end_timestamp_ns = _mode_interval(
        state_messages,
        'fluxline_follow',
        pieps_messages[0].timestamp_ns,
        pieps_messages[-1].timestamp_ns,
    )
    bag_start_timestamp_ns = min(
        pieps_messages[0].timestamp_ns,
        position_messages[0].timestamp_ns,
        min(message.timestamp_ns for message in state_messages),
    )
    heading_values = _latest_values(
        pieps_messages,
        position_messages,
        lambda message: {
            'heading_rad': float(message.message.heading),
            'heading_valid': bool(getattr(message.message, 'heading_good_for_control', True)),
        },
    )
    yaw_rate_values = _latest_values(
        pieps_messages, command_messages, lambda message: float(message.message.angular.z)
    )
    state_values = _latest_values(
        pieps_messages, state_messages, lambda message: str(message.message.data)
    )
    rows = []
    for message in pieps_messages:
        if not start_timestamp_ns <= message.timestamp_ns <= end_timestamp_ns:
            continue
        measurement = message.message
        angle_valid = bool(measurement.angle_valid)
        angle = float(measurement.angle)
        rows.append(
            {
                'timestamp_ns': message.timestamp_ns,
                'time_s': _message_time_s(message, bag_start_timestamp_ns),
                'uav_heading_rad': (
                    heading_values[message.timestamp_ns]['heading_rad']
                    if heading_values[message.timestamp_ns]
                    else None
                ),
                'heading_valid': (
                    heading_values[message.timestamp_ns]['heading_valid']
                    if heading_values[message.timestamp_ns]
                    else False
                ),
                'arva_heading_offset_rad': angle,
                'angle_valid': angle_valid,
                'heading_error_rad': angle if angle_valid else None,
                'commanded_yaw_rate_radps': yaw_rate_values[message.timestamp_ns],
                'mode_state': state_values[message.timestamp_ns],
                'controller_active': state_values[message.timestamp_ns] == 'fluxline_follow',
            }
        )
    if not rows:
        raise ProtocolError('no ARVA samples fall within the fluxline_follow interval')

    analysis_config = _mapping(config.get('analysis'), 'analysis')
    settling_band_rad = math.radians(
        _numeric(analysis_config.get('settling_band_deg', 5.0), 'analysis.settling_band_deg')
    )
    steady_state_window_s = _numeric(
        analysis_config.get('steady_state_window_s', 5.0),
        'analysis.steady_state_window_s',
    )
    response = _response_metrics(
        rows, 'heading_error_rad', settling_band_rad, steady_state_window_s
    )
    event_messages = event_messages or []
    event_offsets = _event_offsets(event_messages, bag_start_timestamp_ns)
    end_of_bag_s = rows[-1]['time_s']
    segment_summaries = []
    raw_segments = analysis_config.get('segments', [])
    if not isinstance(raw_segments, list):
        raise ProtocolError('analysis.segments must be a list')
    for index, raw_segment in enumerate(raw_segments, start=1):
        segment = _mapping(raw_segment, f'analysis.segments[{index}]')
        start_offset_s = _interval_bound(
            segment,
            'start_offset_s',
            'start_event',
            event_offsets,
            f'analysis.segments[{index}]',
        )
        end_offset_s = _interval_bound(
            segment,
            'end_offset_s',
            'end_event',
            event_offsets,
            f'analysis.segments[{index}]',
            end_of_bag_s,
        )
        if end_offset_s <= start_offset_s:
            raise ProtocolError(f'analysis.segments[{index}] must have a positive time range')
        segment_rows = [
            row for row in rows if start_offset_s <= row['time_s'] <= end_offset_s
        ]
        segment_response = _response_metrics(
            segment_rows, 'heading_error_rad', settling_band_rad, steady_state_window_s
        )
        segment_summaries.append(
            {
                'id': str(segment.get('id', f'segment_{index:02d}')),
                'sample_count': len(segment_rows),
                'heading_error_rad': segment_response['error'],
                'settling_time_s': segment_response['settling_time_s'],
                'overshoot_rad': segment_response['overshoot'],
                'oscillation_crossings': segment_response['oscillation_crossings'],
                'steady_state_error_rad': segment_response['steady_state_error'],
            }
        )
    return rows, {
        'sample_count': len(rows),
        'heading_error_rad': response['error'],
        'settling_time_s': response['settling_time_s'],
        'overshoot_rad': response['overshoot'],
        'oscillation_crossings': response['oscillation_crossings'],
        'steady_state_error_rad': response['steady_state_error'],
        'segments': segment_summaries,
    }


def _plot_heading(rows: list[dict[str, Any]], results_directory: Path) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    figure, axes = pyplot.subplots(2, 1, figsize=(10, 7), sharex=True)
    valid_rows = [row for row in rows if row['heading_error_rad'] is not None]
    axes[0].plot(
        [row['time_s'] for row in valid_rows],
        [math.degrees(row['heading_error_rad']) for row in valid_rows],
    )
    axes[0].set_ylabel('ARVA heading offset [deg]')
    axes[0].grid(True)
    command_rows = [row for row in rows if row['commanded_yaw_rate_radps'] is not None]
    axes[1].plot(
        [row['time_s'] for row in command_rows],
        [row['commanded_yaw_rate_radps'] for row in command_rows],
    )
    axes[1].set_xlabel('Time from bag start [s]')
    axes[1].set_ylabel('Commanded yaw rate [rad/s]')
    axes[1].grid(True)
    figure.tight_layout()
    figure.savefig(results_directory / 'heading_controller.png', dpi=200)
    pyplot.close(figure)


def _ground_truth_xy(config: dict[str, Any], metadata: dict[str, Any]) -> tuple[float, float]:
    """Load surveyed transmitter XY coordinates from run metadata or configuration."""
    metadata_position = metadata.get('transmitter_position')
    candidates = []
    if isinstance(metadata_position, dict):
        candidates.append(metadata_position.get('meters'))
    elif metadata_position is not None:
        candidates.append(metadata_position)
    test_config = _mapping(config.get('test'), 'test')
    candidates.append(test_config.get('transmitter_position_ned_m'))

    for value in candidates:
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            continue
        north, east = value[0], value[1]
        if (
            isinstance(north, bool)
            or isinstance(east, bool)
            or not isinstance(north, (int, float))
            or not isinstance(east, (int, float))
        ):
            continue
        if math.isfinite(north) and math.isfinite(east):
            return float(north), float(east)
    raise ProtocolError(
        'provide a finite transmitter_position in metadata or test.transmitter_position_ned_m'
    )


def _next_mode_after(
    state_messages: list[BagMessage], state: str, start_timestamp_ns: int
) -> str | None:
    for message in sorted(state_messages, key=lambda item: item.timestamp_ns):
        if message.timestamp_ns > start_timestamp_ns:
            return str(message.message.data)
    return None


def _validity_transitions(values: list[bool]) -> dict[str, int]:
    """Count signal loss and recovery edges in a validity sequence."""
    losses = 0
    recoveries = 0
    for previous, current in zip(values, values[1:]):
        if previous and not current:
            losses += 1
        elif not previous and current:
            recoveries += 1
    return {'losses': losses, 'recoveries': recoveries}


def _fluxline_rows(
    position_messages: list[BagMessage],
    pieps_messages: list[BagMessage],
    command_messages: list[BagMessage],
    state_messages: list[BagMessage],
    config: dict[str, Any],
    metadata: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not position_messages:
        raise ProtocolError(f'bag contains no {LOCAL_POSITION_TOPIC} messages')
    transmitter = _ground_truth_xy(config, metadata)
    position_messages = sorted(position_messages, key=lambda item: item.timestamp_ns)
    start_timestamp_ns, end_timestamp_ns = _mode_interval(
        state_messages,
        'fluxline_follow',
        position_messages[0].timestamp_ns,
        position_messages[-1].timestamp_ns,
    )
    transition_state = _next_mode_after(state_messages, 'fluxline_follow', start_timestamp_ns)
    bag_start_timestamp_ns = min(
        position_messages[0].timestamp_ns,
        min(message.timestamp_ns for message in state_messages),
    )
    pieps_values = _latest_values(
        position_messages,
        pieps_messages,
        lambda message: {
            'distance_m': float(message.message.distance),
            'angle_rad': float(message.message.angle),
            'distance_valid': bool(message.message.distance_valid),
            'angle_valid': bool(message.message.angle_valid),
        },
    )
    command_values = _latest_values(
        position_messages,
        command_messages,
        lambda message: {
            'north_velocity_mps': float(message.message.linear.x),
            'east_velocity_mps': float(message.message.linear.y),
            'yaw_rate_radps': float(message.message.angular.z),
        },
    )
    rows = []
    for message in position_messages:
        if not start_timestamp_ns <= message.timestamp_ns <= end_timestamp_ns:
            continue
        position = message.message
        point = (float(position.x), float(position.y))
        arva = pieps_values[message.timestamp_ns]
        command = command_values[message.timestamp_ns]
        rows.append(
            {
                'timestamp_ns': message.timestamp_ns,
                'time_s': _message_time_s(message, bag_start_timestamp_ns),
                'north_m': point[0],
                'east_m': point[1],
                'uav_heading_rad': getattr(position, 'heading', None),
                'heading_valid': bool(getattr(position, 'heading_good_for_control', True)),
                'distance_to_transmitter_m': xy_distance(point, transmitter),
                'arva_distance_m': arva['distance_m'] if arva else None,
                'arva_angle_rad': arva['angle_rad'] if arva else None,
                'distance_valid': arva['distance_valid'] if arva else False,
                'angle_valid': arva['angle_valid'] if arva else False,
                'commanded_north_velocity_mps': command['north_velocity_mps'] if command else None,
                'commanded_east_velocity_mps': command['east_velocity_mps'] if command else None,
                'commanded_yaw_rate_radps': command['yaw_rate_radps'] if command else None,
            }
        )
    if not rows:
        raise ProtocolError('no local-position samples fall within the fluxline_follow interval')

    validity = _validity_transitions([row['distance_valid'] for row in rows])
    start_distance = rows[0]['distance_to_transmitter_m']
    path_length_m = path_length((row['north_m'], row['east_m']) for row in rows)
    transitioned_to_fine_search = transition_state == 'fine_search'
    metrics = {
        'success': transitioned_to_fine_search,
        'convergence_time_s': (
            (end_timestamp_ns - start_timestamp_ns) / 1_000_000_000.0
            if transitioned_to_fine_search
            else None
        ),
        'flight_path_length_m': path_length_m,
        'path_length_to_initial_distance_ratio': (
            path_length_m / start_distance if start_distance > 0.0 else None
        ),
        'minimum_distance_to_transmitter_m': min(
            row['distance_to_transmitter_m'] for row in rows
        ),
        'distance_at_transition_m': rows[-1]['distance_to_transmitter_m'],
        'signal_availability': {
            'distance_valid_rate': sum(row['distance_valid'] for row in rows) / len(rows),
            'angle_valid_rate': sum(row['angle_valid'] for row in rows) / len(rows),
            **validity,
        },
    }
    return rows, metrics


def _plot_fluxline(
    rows: list[dict[str, Any]], config: dict[str, Any], metadata: dict[str, Any],
    results_directory: Path,
) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    transmitter = _ground_truth_xy(config, metadata)
    figure, axes = pyplot.subplots(2, 1, figsize=(9, 9))
    axes[0].plot([row['east_m'] for row in rows], [row['north_m'] for row in rows], label='UAV trajectory')
    axes[0].scatter([transmitter[1]], [transmitter[0]], marker='x', s=80, label='Transmitter')
    axes[0].set_xlabel('East [m]')
    axes[0].set_ylabel('North [m]')
    axes[0].set_aspect('equal', adjustable='box')
    axes[0].grid(True)
    axes[0].legend()
    axes[1].plot(
        [row['time_s'] for row in rows],
        [row['arva_distance_m'] for row in rows],
        label='ARVA distance',
    )
    axes[1].set_xlabel('Time from bag start [s]')
    axes[1].set_ylabel('ARVA distance [m]')
    axes[1].grid(True)
    figure.tight_layout()
    figure.savefig(results_directory / 'fluxline_follow.png', dpi=200)
    pyplot.close(figure)


def _fine_search_rows(
    position_messages: list[BagMessage],
    pieps_messages: list[BagMessage],
    estimate_messages: list[BagMessage],
    completion_messages: list[BagMessage],
    fine_state_messages: list[BagMessage],
    mode_state_messages: list[BagMessage],
    config: dict[str, Any],
    metadata: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not position_messages:
        raise ProtocolError(f'bag contains no {LOCAL_POSITION_TOPIC} messages')
    transmitter = _ground_truth_xy(config, metadata)
    position_messages = sorted(position_messages, key=lambda item: item.timestamp_ns)
    start_timestamp_ns, end_timestamp_ns = _mode_interval(
        mode_state_messages,
        'fine_search',
        position_messages[0].timestamp_ns,
        position_messages[-1].timestamp_ns,
    )
    bag_start_timestamp_ns = min(
        position_messages[0].timestamp_ns,
        min(message.timestamp_ns for message in mode_state_messages),
    )
    pieps_values = _latest_values(
        position_messages,
        pieps_messages,
        lambda message: {
            'distance_m': float(message.message.distance),
            'distance_valid': bool(message.message.distance_valid),
        },
    )
    estimate_values = _latest_values(
        position_messages,
        estimate_messages,
        lambda message: (
            float(message.message.point.x),
            float(message.message.point.y),
        ),
    )
    completion_values = _latest_values(
        position_messages,
        completion_messages,
        lambda message: bool(message.message.data),
    )
    state_values = _latest_values(
        position_messages,
        fine_state_messages,
        lambda message: str(message.message.data),
    )
    rows = []
    for message in position_messages:
        if not start_timestamp_ns <= message.timestamp_ns <= end_timestamp_ns:
            continue
        position = message.message
        point = (float(position.x), float(position.y))
        estimate = estimate_values[message.timestamp_ns]
        arva = pieps_values[message.timestamp_ns]
        rows.append(
            {
                'timestamp_ns': message.timestamp_ns,
                'time_s': _message_time_s(message, bag_start_timestamp_ns),
                'north_m': point[0],
                'east_m': point[1],
                'distance_to_transmitter_m': xy_distance(point, transmitter),
                'arva_distance_m': arva['distance_m'] if arva else None,
                'distance_valid': arva['distance_valid'] if arva else False,
                'fine_search_state': state_values[message.timestamp_ns],
                'estimated_north_m': estimate[0] if estimate else None,
                'estimated_east_m': estimate[1] if estimate else None,
                'search_completed': completion_values[message.timestamp_ns] is True,
            }
        )
    if not rows:
        raise ProtocolError('no local-position samples fall within the fine_search interval')

    estimate_messages = [
        message for message in estimate_messages
        if start_timestamp_ns <= message.timestamp_ns <= end_timestamp_ns
    ]
    final_estimate = None
    if estimate_messages:
        final_point = estimate_messages[-1].message.point
        final_estimate = (float(final_point.x), float(final_point.y))
    completion_timestamp_ns = next(
        (
            message.timestamp_ns
            for message in sorted(completion_messages, key=lambda item: item.timestamp_ns)
            if start_timestamp_ns <= message.timestamp_ns <= end_timestamp_ns
            and bool(message.message.data)
        ),
        None,
    )
    localization_error = (
        xy_distance(final_estimate, transmitter) if final_estimate is not None else None
    )
    success = completion_timestamp_ns is not None and final_estimate is not None
    metrics = {
        'success': success,
        'search_duration_s': (
            (completion_timestamp_ns - start_timestamp_ns) / 1_000_000_000.0
            if completion_timestamp_ns is not None
            else None
        ),
        'estimated_position_ned_xy_m': list(final_estimate) if final_estimate else None,
        'localization_error_xy_m': localization_error,
        'minimum_uav_distance_to_transmitter_m': min(
            row['distance_to_transmitter_m'] for row in rows
        ),
        'flight_path_length_m': path_length(
            (row['north_m'], row['east_m']) for row in rows
        ),
        'distance_valid_rate': sum(row['distance_valid'] for row in rows) / len(rows),
    }
    return rows, metrics


def _plot_fine_search(
    rows: list[dict[str, Any]], config: dict[str, Any], metadata: dict[str, Any],
    results_directory: Path,
) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    transmitter = _ground_truth_xy(config, metadata)
    estimates = [
        row for row in rows
        if row['estimated_north_m'] is not None and row['estimated_east_m'] is not None
    ]
    figure, axis = pyplot.subplots(figsize=(8, 7))
    axis.plot([row['east_m'] for row in rows], [row['north_m'] for row in rows], label='UAV trajectory')
    axis.scatter([transmitter[1]], [transmitter[0]], marker='x', s=80, label='Transmitter')
    if estimates:
        estimate = estimates[-1]
        axis.scatter(
            [estimate['estimated_east_m']], [estimate['estimated_north_m']],
            marker='o', facecolors='none', edgecolors='tab:red', s=80,
            label='Final estimate',
        )
    axis.set_xlabel('East [m]')
    axis.set_ylabel('North [m]')
    axis.set_aspect('equal', adjustable='box')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(results_directory / 'fine_search.png', dpi=200)
    pyplot.close(figure)


def _full_search_rows(
    position_messages: list[BagMessage],
    pieps_messages: list[BagMessage],
    estimate_messages: list[BagMessage],
    completion_messages: list[BagMessage],
    mode_state_messages: list[BagMessage],
    config: dict[str, Any],
    metadata: dict[str, Any],
) -> tuple[list[dict[str, Any]], list[dict[str, Any]], dict[str, Any]]:
    if not position_messages:
        raise ProtocolError(f'bag contains no {LOCAL_POSITION_TOPIC} messages')
    if not mode_state_messages:
        raise ProtocolError(f'bag contains no {MODE_STATE_TOPIC} messages')
    transmitter = _ground_truth_xy(config, metadata)
    position_messages = sorted(position_messages, key=lambda item: item.timestamp_ns)
    transitions = _state_transitions(mode_state_messages)
    if not transitions:
        raise ProtocolError('no Mode Executor transitions were recorded')
    transition_times = {state: timestamp for timestamp, state in transitions}
    waypoint_time = transition_times.get('waypoint_follow')
    mission_start_timestamp_ns = transition_times.get('takeoff', waypoint_time)
    if mission_start_timestamp_ns is None:
        mission_start_timestamp_ns = transitions[0][0]
    mission_end_timestamp_ns = next(
        (
            timestamp_ns
            for timestamp_ns, state in transitions
            if state == 'wait_until_disarmed' and timestamp_ns >= mission_start_timestamp_ns
        ),
        None,
    )
    bag_start_timestamp_ns = min(
        position_messages[0].timestamp_ns, transitions[0][0]
    )
    mode_values = _latest_values(
        position_messages, mode_state_messages, lambda message: str(message.message.data)
    )
    pieps_values = _latest_values(
        position_messages,
        pieps_messages,
        lambda message: {
            'distance_m': float(message.message.distance),
            'angle_rad': float(message.message.angle),
            'distance_valid': bool(message.message.distance_valid),
            'angle_valid': bool(message.message.angle_valid),
        },
    )
    estimate_values = _latest_values(
        position_messages,
        estimate_messages,
        lambda message: (
            float(message.message.point.x),
            float(message.message.point.y),
        ),
    )
    completion_values = _latest_values(
        position_messages,
        completion_messages,
        lambda message: bool(message.message.data),
    )
    rows = []
    for message in position_messages:
        if message.timestamp_ns < mission_start_timestamp_ns:
            continue
        if mission_end_timestamp_ns is not None and message.timestamp_ns > mission_end_timestamp_ns:
            continue
        position = message.message
        point = (float(position.x), float(position.y))
        arva = pieps_values[message.timestamp_ns]
        estimate = estimate_values[message.timestamp_ns]
        rows.append(
            {
                'timestamp_ns': message.timestamp_ns,
                'time_s': _message_time_s(message, bag_start_timestamp_ns),
                'north_m': point[0],
                'east_m': point[1],
                'mode_state': mode_values[message.timestamp_ns],
                'arva_distance_m': arva['distance_m'] if arva else None,
                'arva_angle_rad': arva['angle_rad'] if arva else None,
                'distance_valid': arva['distance_valid'] if arva else False,
                'angle_valid': arva['angle_valid'] if arva else False,
                'estimated_north_m': estimate[0] if estimate else None,
                'estimated_east_m': estimate[1] if estimate else None,
                'search_completed': completion_values[message.timestamp_ns] is True,
            }
        )

    transition_rows = [
        {
            'timestamp_ns': timestamp_ns,
            'time_s': (timestamp_ns - bag_start_timestamp_ns) / 1_000_000_000.0,
            'mode_state': state,
        }
        for timestamp_ns, state in transitions
    ]
    fluxline_time = transition_times.get('fluxline_follow')
    fine_search_time = transition_times.get('fine_search')
    completion_timestamp_ns = next(
        (
            message.timestamp_ns
            for message in sorted(completion_messages, key=lambda item: item.timestamp_ns)
            if fine_search_time is not None
            and message.timestamp_ns >= fine_search_time
            and (
                mission_end_timestamp_ns is None
                or message.timestamp_ns <= mission_end_timestamp_ns
            )
            and bool(message.message.data)
        ),
        None,
    )
    final_estimate = None
    valid_estimates = [
        message
        for message in estimate_messages
        if fine_search_time is not None
        and message.timestamp_ns >= fine_search_time
        and (
            mission_end_timestamp_ns is None
            or message.timestamp_ns <= mission_end_timestamp_ns
        )
        and (completion_timestamp_ns is None or message.timestamp_ns <= completion_timestamp_ns)
    ]
    if valid_estimates:
        final_point = sorted(valid_estimates, key=lambda item: item.timestamp_ns)[-1].message.point
        final_estimate = (float(final_point.x), float(final_point.y))
    localization_error = (
        xy_distance(final_estimate, transmitter) if final_estimate is not None else None
    )
    ordered_mode_progression = (
        waypoint_time is not None
        and fluxline_time is not None
        and fine_search_time is not None
        and waypoint_time <= fluxline_time <= fine_search_time
    )
    analysis_config = _mapping(config.get('analysis'), 'analysis')
    require_land_transition = bool(analysis_config.get('require_land_transition', True))
    land_timestamp_ns = next(
        (
            timestamp_ns
            for timestamp_ns, state in transitions
            if state == 'land'
            and completion_timestamp_ns is not None
            and timestamp_ns >= completion_timestamp_ns
        ),
        None,
    )
    mission_completion_success = (
        completion_timestamp_ns is not None
        and (not require_land_transition or land_timestamp_ns is not None)
    )
    success = (
        ordered_mode_progression
        and mission_completion_success
        and final_estimate is not None
    )
    first_detection_timestamp_ns = next(
        (
            message.timestamp_ns
            for message in sorted(pieps_messages, key=lambda item: item.timestamp_ns)
            if message.timestamp_ns >= mission_start_timestamp_ns
            and (
                mission_end_timestamp_ns is None
                or message.timestamp_ns <= mission_end_timestamp_ns
            )
            and bool(message.message.distance_valid)
        ),
        None,
    )
    mission_start_bag_offset_s = (
        mission_start_timestamp_ns - bag_start_timestamp_ns
    ) / 1_000_000_000.0

    def mission_elapsed_s(timestamp_ns: int) -> float:
        return (timestamp_ns - mission_start_timestamp_ns) / 1_000_000_000.0

    metrics = {
        'success': success,
        'mode_progression_success': ordered_mode_progression,
        'mission_completion_success': mission_completion_success,
        'land_transition_time_s': (
            mission_elapsed_s(land_timestamp_ns)
            if land_timestamp_ns is not None
            else None
        ),
        'mission_start_time_s': mission_start_bag_offset_s,
        'first_valid_arva_detection_time_s': (
            mission_elapsed_s(first_detection_timestamp_ns)
            if first_detection_timestamp_ns is not None
            else None
        ),
        'fluxline_activation_time_s': (
            mission_elapsed_s(fluxline_time)
            if fluxline_time is not None
            else None
        ),
        'fine_search_activation_time_s': (
            mission_elapsed_s(fine_search_time)
            if fine_search_time is not None
            else None
        ),
        'final_localization_time_s': (
            mission_elapsed_s(completion_timestamp_ns)
            if completion_timestamp_ns is not None
            else None
        ),
        'total_search_time_s': (
            mission_elapsed_s(completion_timestamp_ns)
            if completion_timestamp_ns is not None
            else None
        ),
        'coverage_search_time_s': (
            (fluxline_time - waypoint_time) / 1_000_000_000.0
            if waypoint_time is not None and fluxline_time is not None
            else None
        ),
        'fluxline_follow_duration_s': (
            (fine_search_time - fluxline_time) / 1_000_000_000.0
            if fluxline_time is not None and fine_search_time is not None
            else None
        ),
        'fine_search_duration_s': (
            (completion_timestamp_ns - fine_search_time) / 1_000_000_000.0
            if completion_timestamp_ns is not None and fine_search_time is not None
            else None
        ),
        'estimated_position_ned_xy_m': list(final_estimate) if final_estimate else None,
        'localization_error_xy_m': localization_error,
        'total_path_length_m': path_length(
            (row['north_m'], row['east_m']) for row in rows
        ),
        'mode_transition_sequence': [state for _, state in transitions],
    }
    return rows, transition_rows, metrics


def _plot_full_search(
    rows: list[dict[str, Any]], transition_rows: list[dict[str, Any]],
    config: dict[str, Any], metadata: dict[str, Any], results_directory: Path,
) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    transmitter = _ground_truth_xy(config, metadata)
    test_config = _mapping(config.get('test'), 'test')
    coverage = test_config.get('planned_coverage_waypoints_ned_m')
    figure, axis = pyplot.subplots(figsize=(8, 7))
    for state in sorted({row['mode_state'] for row in rows if row['mode_state']}):
        state_rows = [row for row in rows if row['mode_state'] == state]
        axis.plot(
            [row['east_m'] for row in state_rows],
            [row['north_m'] for row in state_rows],
            label=state,
        )
    if coverage:
        waypoints = parse_xy_waypoints(coverage, 'test.planned_coverage_waypoints_ned_m')
        axis.plot(
            [point[1] for point in waypoints], [point[0] for point in waypoints],
            '--', color='black', label='Planned coverage',
        )
    axis.scatter([transmitter[1]], [transmitter[0]], marker='x', s=80, label='Transmitter')
    axis.set_xlabel('East [m]')
    axis.set_ylabel('North [m]')
    axis.set_aspect('equal', adjustable='box')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(results_directory / 'full_search_trajectory.png', dpi=200)
    pyplot.close(figure)

    figure, axis = pyplot.subplots(figsize=(10, 3))
    for transition in transition_rows:
        axis.axvline(transition['time_s'])
        axis.text(
            transition['time_s'], 0.5, transition['mode_state'], rotation=90,
            verticalalignment='center', horizontalalignment='right',
        )
    axis.set_xlabel('Time from bag start [s]')
    axis.set_yticks([])
    axis.grid(True, axis='x')
    figure.tight_layout()
    figure.savefig(results_directory / 'full_search_timeline.png', dpi=200)
    pyplot.close(figure)


def _waypoint_turns(
    waypoints: list[tuple[float, float]], threshold_deg: float
) -> list[tuple[int, tuple[float, float]]]:
    turns = []
    for index in range(1, len(waypoints) - 1):
        previous = waypoints[index - 1]
        current = waypoints[index]
        following = waypoints[index + 1]
        incoming = (current[0] - previous[0], current[1] - previous[1])
        outgoing = (following[0] - current[0], following[1] - current[1])
        incoming_length = math.hypot(*incoming)
        outgoing_length = math.hypot(*outgoing)
        if incoming_length == 0.0 or outgoing_length == 0.0:
            continue
        cosine = max(
            -1.0,
            min(1.0, (incoming[0] * outgoing[0] + incoming[1] * outgoing[1]) / (incoming_length * outgoing_length)),
        )
        if math.degrees(math.acos(cosine)) >= threshold_deg:
            turns.append((index, current))
    return turns


def _waypoint_rows(
    position_messages: list[BagMessage],
    state_messages: list[BagMessage],
    config: dict[str, Any],
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    if not position_messages:
        raise ProtocolError(f'bag contains no {LOCAL_POSITION_TOPIC} messages')
    test_config = _mapping(config.get('test'), 'test')
    waypoints = parse_xy_waypoints(
        test_config.get('planned_waypoints_ned_m'), 'test.planned_waypoints_ned_m'
    )
    position_messages = sorted(position_messages, key=lambda item: item.timestamp_ns)
    start_timestamp_ns, end_timestamp_ns = _mode_interval(
        state_messages,
        'waypoint_follow',
        position_messages[0].timestamp_ns,
        position_messages[-1].timestamp_ns,
    )
    bag_start_timestamp_ns = min(
        position_messages[0].timestamp_ns,
        min(message.timestamp_ns for message in state_messages),
    )
    rows = []
    for message in position_messages:
        if not start_timestamp_ns <= message.timestamp_ns <= end_timestamp_ns:
            continue
        position = message.message
        point = (float(position.x), float(position.y))
        rows.append(
            {
                'timestamp_ns': message.timestamp_ns,
                'time_s': _message_time_s(message, bag_start_timestamp_ns),
                'north_m': point[0],
                'east_m': point[1],
                'heading_rad': getattr(position, 'heading', None),
                'north_velocity_mps': getattr(position, 'vx', None),
                'east_velocity_mps': getattr(position, 'vy', None),
                'cross_track_error_m': point_to_polyline_distance(point, waypoints),
            }
        )
    if not rows:
        raise ProtocolError('no local-position samples fall within the waypoint_follow interval')

    analysis_config = _mapping(config.get('analysis'), 'analysis')
    turn_threshold_deg = _numeric(
        analysis_config.get('turn_threshold_deg', 60.0), 'analysis.turn_threshold_deg'
    )
    turn_window_m = _numeric(
        analysis_config.get('turn_window_m', 5.0), 'analysis.turn_window_m'
    )
    turn_summaries = []
    for index, waypoint in _waypoint_turns(waypoints, turn_threshold_deg):
        near_turn = [
            row for row in rows
            if xy_distance((row['north_m'], row['east_m']), waypoint) <= turn_window_m
        ]
        turn_summaries.append(
            {
                'waypoint_index': index,
                'waypoint_ned_m': list(waypoint),
                'sample_count': len(near_turn),
                'maximum_path_error_m': (
                    max(row['cross_track_error_m'] for row in near_turn)
                    if near_turn
                    else None
                ),
            }
        )

    metrics = {
        'sample_count': len(rows),
        'actual_path_length_m': path_length(
            (row['north_m'], row['east_m']) for row in rows
        ),
        'trajectory_error_m': error_metrics(
            row['cross_track_error_m'] for row in rows
        ),
        'turn_neighborhoods': turn_summaries,
    }
    return rows, metrics


def _plot_waypoint(
    rows: list[dict[str, Any]], config: dict[str, Any], results_directory: Path
) -> None:
    import matplotlib

    matplotlib.use('Agg')
    import matplotlib.pyplot as pyplot

    waypoints = parse_xy_waypoints(
        _mapping(config.get('test'), 'test').get('planned_waypoints_ned_m'),
        'test.planned_waypoints_ned_m',
    )
    figure, axis = pyplot.subplots(figsize=(8, 7))
    axis.plot(
        [point[1] for point in waypoints],
        [point[0] for point in waypoints],
        '--o',
        label='Planned path',
    )
    axis.plot(
        [row['east_m'] for row in rows],
        [row['north_m'] for row in rows],
        label='Flown path',
    )
    axis.set_xlabel('East [m]')
    axis.set_ylabel('North [m]')
    axis.set_aspect('equal', adjustable='box')
    axis.grid(True)
    axis.legend()
    figure.tight_layout()
    figure.savefig(results_directory / 'waypoint_trajectory.png', dpi=200)
    pyplot.close(figure)


def _run_directory(path: Path) -> Path:
    resolved_path = path.expanduser().resolve()
    if not (resolved_path / 'config.yaml').is_file():
        raise ProtocolError(f'run directory has no config.yaml: {resolved_path}')
    if not (resolved_path / 'metadata.yaml').is_file():
        raise ProtocolError(f'run directory has no metadata.yaml: {resolved_path}')
    return resolved_path


def _bag_directory(run_directory: Path, metadata: dict[str, Any]) -> Path:
    bag_path = metadata.get('bag_path', 'bag')
    if not isinstance(bag_path, str):
        raise ProtocolError('metadata bag_path must be a relative path string')
    candidate = (run_directory / bag_path).resolve()
    try:
        candidate.relative_to(run_directory)
    except ValueError as error:
        raise ProtocolError('metadata bag_path must remain inside the run directory') from error
    return candidate


def analyze_run(run_directory: Path) -> Path:
    """Analyze one recorded run and return its results directory."""
    resolved_run_directory = _run_directory(run_directory)
    config = load_yaml(resolved_run_directory / 'config.yaml')
    metadata = load_yaml(resolved_run_directory / 'metadata.yaml')
    analysis_config = _mapping(config.get('analysis'), 'analysis')
    kind = analysis_config.get('kind')
    if not isinstance(kind, str):
        raise ProtocolError('analysis.kind must be a string')

    bag_directory = _bag_directory(resolved_run_directory, metadata)
    results_directory = resolved_run_directory / 'results'
    results_directory.mkdir(parents=True, exist_ok=True)

    if kind == 'arva_characterization':
        topics = [PIEPS_TOPIC]
        if _uses_event_markers(analysis_config, 'samples'):
            topics.append(EVENT_TOPIC)
        messages = read_messages(bag_directory, topics)
        rows, metrics = _arva_rows(
            [message for message in messages if message.topic == PIEPS_TOPIC],
            analysis_config,
            [message for message in messages if message.topic == EVENT_TOPIC],
        )
        _write_csv(
            results_directory / 'arva_measurements.csv',
            [
                'sample_id', 'timestamp_ns', 'time_s', 'expected_distance_m',
                'distance_m', 'distance_valid', 'distance_error_m',
                'expected_angle_rad', 'angle_rad', 'angle_valid', 'angle_error_rad',
            ],
            rows,
        )
        plot_warning = _try_plot(lambda: _plot_arva(rows, results_directory))
    elif kind == 'terrain_follow':
        topics = [LOCAL_POSITION_TOPIC, TERRAIN_COMMAND_TOPIC]
        if _uses_event_markers(analysis_config, 'segments'):
            topics.append(EVENT_TOPIC)
        messages = read_messages(bag_directory, topics)
        rows, metrics = _terrain_rows(
            [message for message in messages if message.topic == LOCAL_POSITION_TOPIC],
            [message for message in messages if message.topic == TERRAIN_COMMAND_TOPIC],
            analysis_config,
            [message for message in messages if message.topic == EVENT_TOPIC],
        )
        _write_csv(
            results_directory / 'terrain_following.csv',
            [
                'segment_id', 'timestamp_ns', 'time_s', 'commanded_agl_m',
                'measured_agl_m', 'agl_error_m',
                'north_m', 'east_m', 'down_m',
                'measured_vertical_velocity_ned_mps',
                'commanded_vertical_velocity_ned_mps',
            ],
            rows,
        )
        plot_warning = _try_plot(lambda: _plot_terrain(rows, results_directory))
    elif kind == 'waypoint_follow':
        messages = read_messages(
            bag_directory, [LOCAL_POSITION_TOPIC, MODE_STATE_TOPIC]
        )
        rows, metrics = _waypoint_rows(
            [message for message in messages if message.topic == LOCAL_POSITION_TOPIC],
            [message for message in messages if message.topic == MODE_STATE_TOPIC],
            config,
        )
        _write_csv(
            results_directory / 'waypoint_trajectory.csv',
            [
                'timestamp_ns', 'time_s', 'north_m', 'east_m', 'heading_rad',
                'north_velocity_mps', 'east_velocity_mps', 'cross_track_error_m',
            ],
            rows,
        )
        plot_warning = _try_plot(lambda: _plot_waypoint(rows, config, results_directory))
    elif kind == 'heading_controller':
        topics = [PIEPS_TOPIC, LOCAL_POSITION_TOPIC, FLUX_COMMAND_TOPIC, MODE_STATE_TOPIC]
        if _uses_event_markers(analysis_config, 'segments'):
            topics.append(EVENT_TOPIC)
        messages = read_messages(bag_directory, topics)
        rows, metrics = _heading_rows(
            [message for message in messages if message.topic == PIEPS_TOPIC],
            [message for message in messages if message.topic == LOCAL_POSITION_TOPIC],
            [message for message in messages if message.topic == FLUX_COMMAND_TOPIC],
            [message for message in messages if message.topic == MODE_STATE_TOPIC],
            config,
            [message for message in messages if message.topic == EVENT_TOPIC],
        )
        _write_csv(
            results_directory / 'heading_controller.csv',
            [
                'timestamp_ns', 'time_s', 'uav_heading_rad', 'arva_heading_offset_rad',
                'heading_valid', 'angle_valid', 'heading_error_rad', 'commanded_yaw_rate_radps',
                'mode_state', 'controller_active',
            ],
            rows,
        )
        plot_warning = _try_plot(lambda: _plot_heading(rows, results_directory))
    elif kind == 'fluxline_follow':
        messages = read_messages(
            bag_directory,
            [LOCAL_POSITION_TOPIC, PIEPS_TOPIC, FLUX_COMMAND_TOPIC, MODE_STATE_TOPIC],
        )
        rows, metrics = _fluxline_rows(
            [message for message in messages if message.topic == LOCAL_POSITION_TOPIC],
            [message for message in messages if message.topic == PIEPS_TOPIC],
            [message for message in messages if message.topic == FLUX_COMMAND_TOPIC],
            [message for message in messages if message.topic == MODE_STATE_TOPIC],
            config,
            metadata,
        )
        _write_csv(
            results_directory / 'fluxline_follow.csv',
            [
                'timestamp_ns', 'time_s', 'north_m', 'east_m',
                'uav_heading_rad', 'heading_valid',
                'distance_to_transmitter_m', 'arva_distance_m', 'arva_angle_rad',
                'distance_valid', 'angle_valid', 'commanded_north_velocity_mps',
                'commanded_east_velocity_mps', 'commanded_yaw_rate_radps',
            ],
            rows,
        )
        plot_warning = _try_plot(
            lambda: _plot_fluxline(rows, config, metadata, results_directory)
        )
    elif kind == 'fine_search':
        messages = read_messages(
            bag_directory,
            [
                LOCAL_POSITION_TOPIC,
                PIEPS_TOPIC,
                FINE_ESTIMATE_TOPIC,
                FINE_COMPLETION_TOPIC,
                FINE_STATE_TOPIC,
                MODE_STATE_TOPIC,
            ],
        )
        rows, metrics = _fine_search_rows(
            [message for message in messages if message.topic == LOCAL_POSITION_TOPIC],
            [message for message in messages if message.topic == PIEPS_TOPIC],
            [message for message in messages if message.topic == FINE_ESTIMATE_TOPIC],
            [message for message in messages if message.topic == FINE_COMPLETION_TOPIC],
            [message for message in messages if message.topic == FINE_STATE_TOPIC],
            [message for message in messages if message.topic == MODE_STATE_TOPIC],
            config,
            metadata,
        )
        _write_csv(
            results_directory / 'fine_search.csv',
            [
                'timestamp_ns', 'time_s', 'north_m', 'east_m',
                'distance_to_transmitter_m', 'arva_distance_m', 'distance_valid',
                'fine_search_state', 'estimated_north_m', 'estimated_east_m',
                'search_completed',
            ],
            rows,
        )
        plot_warning = _try_plot(
            lambda: _plot_fine_search(rows, config, metadata, results_directory)
        )
    elif kind == 'full_search':
        messages = read_messages(
            bag_directory,
            [
                LOCAL_POSITION_TOPIC,
                PIEPS_TOPIC,
                FINE_ESTIMATE_TOPIC,
                FINE_COMPLETION_TOPIC,
                MODE_STATE_TOPIC,
            ],
        )
        rows, transition_rows, metrics = _full_search_rows(
            [message for message in messages if message.topic == LOCAL_POSITION_TOPIC],
            [message for message in messages if message.topic == PIEPS_TOPIC],
            [message for message in messages if message.topic == FINE_ESTIMATE_TOPIC],
            [message for message in messages if message.topic == FINE_COMPLETION_TOPIC],
            [message for message in messages if message.topic == MODE_STATE_TOPIC],
            config,
            metadata,
        )
        _write_csv(
            results_directory / 'full_search.csv',
            [
                'timestamp_ns', 'time_s', 'north_m', 'east_m', 'mode_state',
                'arva_distance_m', 'arva_angle_rad', 'distance_valid', 'angle_valid',
                'estimated_north_m', 'estimated_east_m', 'search_completed',
            ],
            rows,
        )
        _write_csv(
            results_directory / 'mode_transitions.csv',
            ['timestamp_ns', 'time_s', 'mode_state'], transition_rows,
        )
        plot_warning = _try_plot(
            lambda: _plot_full_search(
                rows, transition_rows, config, metadata, results_directory
            )
        )
    else:
        raise ProtocolError(
            f'analysis kind {kind!r} is not implemented yet; recording remains available.'
        )

    summary = {
        'test_type': metadata.get('test_type'),
        'run_id': metadata.get('run_id'),
        'bag_path': str(bag_directory.relative_to(resolved_run_directory)),
        'analysis_kind': kind,
        'metrics': metrics,
    }
    if plot_warning:
        summary['plot_warning'] = plot_warning
    dump_yaml(results_directory / 'summary.yaml', summary)
    return results_directory


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Analyze a recorded ARVA test run without affecting the vehicle.'
    )
    parser.add_argument('--run-dir', required=True, help='Recorded run directory')
    return parser


def analyze_main(arguments: Sequence[str] | None = None) -> int:
    """CLI entry point for post-flight rosbag analysis."""
    parser = _parser()
    arguments_namespace = parser.parse_args(arguments)
    try:
        results_directory = analyze_run(Path(arguments_namespace.run_dir))
    except ProtocolError as error:
        parser.error(str(error))
    print(f'[arva test protocol] analysis completed: {results_directory}')
    return 0


if __name__ == '__main__':
    sys.exit(analyze_main())