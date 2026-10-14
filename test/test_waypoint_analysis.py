from types import SimpleNamespace

from arva_test_protocol.analysis import LOCAL_POSITION_TOPIC, MODE_STATE_TOPIC, _waypoint_rows
from arva_test_protocol.bag_reader import BagMessage


def test_waypoint_rows_uses_mode_interval_and_cross_track_error():
    state_messages = [
        BagMessage(MODE_STATE_TOPIC, 10, SimpleNamespace(data='takeoff')),
        BagMessage(MODE_STATE_TOPIC, 20, SimpleNamespace(data='waypoint_follow')),
        BagMessage(MODE_STATE_TOPIC, 50, SimpleNamespace(data='fluxline_follow')),
    ]
    position_messages = [
        BagMessage(LOCAL_POSITION_TOPIC, 15, SimpleNamespace(x=0.0, y=0.0, heading=0.0, vx=0.0, vy=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 25, SimpleNamespace(x=1.0, y=0.0, heading=0.0, vx=1.0, vy=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 35, SimpleNamespace(x=2.0, y=0.5, heading=0.0, vx=1.0, vy=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 55, SimpleNamespace(x=3.0, y=0.0, heading=0.0, vx=1.0, vy=0.0)),
    ]
    config = {
        'test': {'planned_waypoints_ned_m': [[0.0, 0.0], [3.0, 0.0]]},
        'analysis': {'kind': 'waypoint_follow'},
    }

    rows, metrics = _waypoint_rows(position_messages, state_messages, config)

    assert len(rows) == 2
    assert rows[1]['cross_track_error_m'] == 0.5
    assert metrics['trajectory_error_m']['maximum_absolute_error'] == 0.5