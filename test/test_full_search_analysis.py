from types import SimpleNamespace

from arva_test_protocol.analysis import (
    FINE_COMPLETION_TOPIC,
    FINE_ESTIMATE_TOPIC,
    LOCAL_POSITION_TOPIC,
    MODE_STATE_TOPIC,
    PIEPS_TOPIC,
    _full_search_rows,
)
from arva_test_protocol.bag_reader import BagMessage


def test_full_search_rows_requires_mode_progression_and_localization():
    states = [
        BagMessage(MODE_STATE_TOPIC, 10, SimpleNamespace(data='takeoff')),
        BagMessage(MODE_STATE_TOPIC, 20, SimpleNamespace(data='waypoint_follow')),
        BagMessage(MODE_STATE_TOPIC, 40, SimpleNamespace(data='fluxline_follow')),
        BagMessage(MODE_STATE_TOPIC, 60, SimpleNamespace(data='fine_search')),
        BagMessage(MODE_STATE_TOPIC, 75, SimpleNamespace(data='land')),
        BagMessage(MODE_STATE_TOPIC, 90, SimpleNamespace(data='wait_until_disarmed')),
    ]
    positions = [
        BagMessage(LOCAL_POSITION_TOPIC, 5, SimpleNamespace(x=100.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 25, SimpleNamespace(x=0.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 45, SimpleNamespace(x=1.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 65, SimpleNamespace(x=2.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 80, SimpleNamespace(x=3.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 95, SimpleNamespace(x=100.0, y=0.0)),
    ]
    pieps = [
        BagMessage(PIEPS_TOPIC, 8, SimpleNamespace(distance=9.0, angle=0.0, distance_valid=True, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 35, SimpleNamespace(distance=5.0, angle=0.0, distance_valid=True, angle_valid=True)),
        BagMessage(PIEPS_TOPIC, 95, SimpleNamespace(distance=1.0, angle=0.0, distance_valid=True, angle_valid=True)),
    ]
    estimates = [
        BagMessage(FINE_ESTIMATE_TOPIC, 65, SimpleNamespace(point=SimpleNamespace(x=2.2, y=0.0))),
        BagMessage(FINE_ESTIMATE_TOPIC, 95, SimpleNamespace(point=SimpleNamespace(x=100.0, y=0.0))),
    ]
    completions = [
        BagMessage(FINE_COMPLETION_TOPIC, 70, SimpleNamespace(data=True)),
        BagMessage(FINE_COMPLETION_TOPIC, 95, SimpleNamespace(data=True)),
    ]
    config = {
        'test': {'transmitter_position_ned_m': [2.0, 0.0, 0.0]},
        'analysis': {'kind': 'full_search'},
    }

    rows, transitions, metrics = _full_search_rows(
        positions, pieps, estimates, completions, states, config, {}
    )

    assert len(rows) == 4
    assert len(transitions) == 6
    assert metrics['success'] is True
    assert metrics['mission_completion_success'] is True
    assert metrics['coverage_search_time_s'] == 0.00000002
    assert metrics['mission_start_time_s'] == 0.000000005
    assert metrics['first_valid_arva_detection_time_s'] == 0.000000025
    assert metrics['total_search_time_s'] == 0.00000006
    assert metrics['estimated_position_ned_xy_m'] == [2.2, 0.0]
    assert metrics['total_path_length_m'] == 3.0
    assert metrics['mode_transition_sequence'][-2:] == ['land', 'wait_until_disarmed']