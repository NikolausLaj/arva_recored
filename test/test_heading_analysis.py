import json
from types import SimpleNamespace
import math

import pytest

from arva_test_protocol.analysis import (
    EVENT_TOPIC,
    FLUX_COMMAND_TOPIC,
    LOCAL_POSITION_TOPIC,
    MODE_STATE_TOPIC,
    PIEPS_TOPIC,
    _heading_rows,
)
from arva_test_protocol.bag_reader import BagMessage


@pytest.mark.parametrize('use_markers', [True, False])
def test_heading_rows_reports_settling_and_command_alignment(use_markers):
    states = [
        BagMessage(MODE_STATE_TOPIC, 10, SimpleNamespace(data='waypoint_follow')),
        BagMessage(MODE_STATE_TOPIC, 20, SimpleNamespace(data='fluxline_follow')),
        BagMessage(MODE_STATE_TOPIC, 70, SimpleNamespace(data='fine_search')),
    ]
    positions = [
        BagMessage(LOCAL_POSITION_TOPIC, 20, SimpleNamespace(heading=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 40, SimpleNamespace(heading=0.2)),
    ]
    commands = [
        BagMessage(FLUX_COMMAND_TOPIC, 20, SimpleNamespace(angular=SimpleNamespace(z=-0.5))),
        BagMessage(FLUX_COMMAND_TOPIC, 40, SimpleNamespace(angular=SimpleNamespace(z=0.0))),
    ]
    pieps = [
        BagMessage(PIEPS_TOPIC, 25, SimpleNamespace(angle=0.3, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 45, SimpleNamespace(angle=0.02, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 55, SimpleNamespace(angle=0.01, angle_valid=True)),
    ]
    events = [
        BagMessage(EVENT_TOPIC, 24, SimpleNamespace(data=json.dumps({'event': 'step_start'}))),
        BagMessage(EVENT_TOPIC, 56, SimpleNamespace(data=json.dumps({'event': 'step_end'}))),
    ]
    config = {
        'analysis': {
            'kind': 'heading_controller',
            'settling_band_deg': 5.0,
            'segments': [
                {'id': 'step', 'start_event': 'step_start', 'end_event': 'step_end'}
            ],
        }
    }

    if not use_markers:
        config['analysis']['segments'] = []
        config['analysis']['use_active_mode_interval'] = True
        events = []

    rows, metrics = _heading_rows(pieps, positions, commands, states, config, events)

    assert len(rows) == 3
    assert rows[0]['commanded_yaw_rate_radps'] == -0.5
    assert math.isclose(metrics['settling_time_s'], 0.00000002)
    if use_markers:
        assert metrics['segments'][0]['sample_count'] == 3
    else:
        assert metrics['segments'] == []
