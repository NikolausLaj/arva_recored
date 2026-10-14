from types import SimpleNamespace

from arva_test_protocol.analysis import (
    FLUX_COMMAND_TOPIC,
    LOCAL_POSITION_TOPIC,
    MODE_STATE_TOPIC,
    PIEPS_TOPIC,
    _fluxline_rows,
)
from arva_test_protocol.bag_reader import BagMessage


def test_fluxline_rows_reports_transition_success_and_path_metrics():
    states = [
        BagMessage(MODE_STATE_TOPIC, 10, SimpleNamespace(data='waypoint_follow')),
        BagMessage(MODE_STATE_TOPIC, 20, SimpleNamespace(data='fluxline_follow')),
        BagMessage(MODE_STATE_TOPIC, 50, SimpleNamespace(data='fine_search')),
    ]
    positions = [
        BagMessage(LOCAL_POSITION_TOPIC, 25, SimpleNamespace(x=0.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 35, SimpleNamespace(x=1.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 45, SimpleNamespace(x=2.0, y=0.0)),
    ]
    pieps = [
        BagMessage(PIEPS_TOPIC, 25, SimpleNamespace(distance=4.0, angle=0.1, distance_valid=True, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 35, SimpleNamespace(distance=3.0, angle=0.1, distance_valid=False, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 45, SimpleNamespace(distance=2.0, angle=0.0, distance_valid=True, angle_valid=False)),
    ]
    commands = [
        BagMessage(FLUX_COMMAND_TOPIC, 20, SimpleNamespace(linear=SimpleNamespace(x=1.0, y=0.0), angular=SimpleNamespace(z=0.0))),
    ]
    config = {
        'test': {'transmitter_position_ned_m': [3.0, 0.0, 0.0]},
        'analysis': {'kind': 'fluxline_follow'},
    }

    rows, metrics = _fluxline_rows(positions, pieps, commands, states, config, {})

    assert len(rows) == 3
    assert metrics['success'] is True
    assert metrics['flight_path_length_m'] == 2.0
    assert metrics['signal_availability']['losses'] == 1
    assert metrics['signal_availability']['recoveries'] == 1