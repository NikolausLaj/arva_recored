import json
import math
from types import SimpleNamespace

from arva_test_protocol.analysis import (
    EVENT_TOPIC,
    LOCAL_POSITION_TOPIC,
    PIEPS_TOPIC,
    TERRAIN_COMMAND_TOPIC,
    _arva_rows,
    _terrain_rows,
)
from arva_test_protocol.bag_reader import BagMessage


def _event(timestamp_ns, name):
    return BagMessage(EVENT_TOPIC, timestamp_ns, SimpleNamespace(data=json.dumps({'event': name})))


def test_arva_samples_can_use_operator_event_boundaries():
    pieps = [
        BagMessage(PIEPS_TOPIC, 10, SimpleNamespace(distance=4.0, angle=0.1, distance_valid=True, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 20, SimpleNamespace(distance=3.0, angle=0.0, distance_valid=True, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 30, SimpleNamespace(distance=2.0, angle=0.0, distance_valid=True, angle_valid=True)),
    ]
    events = [_event(15, 'sample_start'), _event(25, 'sample_end')]
    rows, _ = _arva_rows(
        pieps,
        {
            'samples': [
                {
                    'id': 'sample',
                    'start_event': 'sample_start',
                    'end_event': 'sample_end',
                    'expected_distance_m': 3.0,
                    'expected_angle_rad': 0.0,
                }
            ]
        },
        events,
    )

    assert len(rows) == 1
    assert rows[0]['distance_m'] == 3.0


def test_terrain_segments_can_use_operator_event_boundaries():
    positions = [
        BagMessage(LOCAL_POSITION_TOPIC, 10, SimpleNamespace(dist_bottom=1.0, dist_bottom_valid=True, vz=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 20, SimpleNamespace(dist_bottom=1.1, dist_bottom_valid=True, vz=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 30, SimpleNamespace(dist_bottom=1.5, dist_bottom_valid=True, vz=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 40, SimpleNamespace(dist_bottom=1.5, dist_bottom_valid=True, vz=0.0)),
    ]
    commands = [
        BagMessage(TERRAIN_COMMAND_TOPIC, 10, SimpleNamespace(linear=SimpleNamespace(z=0.0))),
    ]
    events = [_event(15, 'step_start'), _event(45, 'step_end')]
    rows, summary = _terrain_rows(
        positions,
        commands,
        {
            'settling_band_m': 0.1,
            'steady_state_window_s': 1.0,
            'segments': [
                {
                    'id': 'step',
                    'start_event': 'step_start',
                    'end_event': 'step_end',
                    'commanded_agl_m': 1.5,
                    'initial_agl_m': 1.0,
                }
            ],
        },
        events,
    )

    assert len(rows) == 3
    assert summary['range_valid_rate'] == 1.0
    assert math.isclose(summary['segments'][0]['settling_time_s'], 0.000000015)