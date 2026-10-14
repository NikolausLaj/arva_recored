from types import SimpleNamespace
import math

from arva_test_protocol.analysis import (
    FINE_COMPLETION_TOPIC,
    FINE_ESTIMATE_TOPIC,
    FINE_STATE_TOPIC,
    LOCAL_POSITION_TOPIC,
    MODE_STATE_TOPIC,
    PIEPS_TOPIC,
    _fine_search_rows,
)
from arva_test_protocol.bag_reader import BagMessage


def test_fine_search_rows_reports_xy_localization_error_and_completion():
    mode_states = [
        BagMessage(MODE_STATE_TOPIC, 10, SimpleNamespace(data='fluxline_follow')),
        BagMessage(MODE_STATE_TOPIC, 20, SimpleNamespace(data='fine_search')),
    ]
    positions = [
        BagMessage(LOCAL_POSITION_TOPIC, 25, SimpleNamespace(x=0.0, y=0.0)),
        BagMessage(LOCAL_POSITION_TOPIC, 35, SimpleNamespace(x=1.0, y=0.0)),
    ]
    pieps = [
        BagMessage(PIEPS_TOPIC, 25, SimpleNamespace(distance=2.0, distance_valid=True)),
        BagMessage(PIEPS_TOPIC, 35, SimpleNamespace(distance=1.0, distance_valid=True)),
    ]
    estimates = [
        BagMessage(
            FINE_ESTIMATE_TOPIC, 30,
            SimpleNamespace(point=SimpleNamespace(x=1.2, y=0.0)),
        ),
    ]
    completions = [BagMessage(FINE_COMPLETION_TOPIC, 35, SimpleNamespace(data=True))]
    states = [BagMessage(FINE_STATE_TOPIC, 25, SimpleNamespace(data='x_search'))]
    config = {
        'test': {'transmitter_position_ned_m': [1.0, 0.0, 0.0]},
        'analysis': {'kind': 'fine_search'},
    }

    rows, metrics = _fine_search_rows(
        positions, pieps, estimates, completions, states, mode_states, config, {}
    )

    assert len(rows) == 2
    assert metrics['success'] is True
    assert metrics['estimated_position_ned_xy_m'] == [1.2, 0.0]
    assert math.isclose(metrics['localization_error_xy_m'], 0.2)