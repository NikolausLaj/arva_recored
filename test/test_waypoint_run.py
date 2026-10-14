from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import pytest

from arva_test_protocol.waypoint_run import waypoint_main
from arva_test_protocol.yaml_utils import load_yaml


_PACKAGE = Path(__file__).parent.parent


def _repository(tmp_path, monkeypatch):
    root = tmp_path / 'repository'
    template = root / 'px4_roscon_25/arva_test_protocol/config/waypoint_follow.yaml'
    template.parent.mkdir(parents=True)
    shutil.copy2(_PACKAGE / 'config/waypoint_follow.yaml', template)
    for filename in load_yaml(template)['artifacts']['parameter_files']:
        destination = root / filename
        destination.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(_PACKAGE.parent.parent / filename, destination)
    source = root / 'px4_roscon_25/arva_master/src/WaypointFollowMode.cpp'
    source.parent.mkdir(parents=True)
    source.write_text('''
void WaypointFollow::initialiseWaypoints()
{
    _trajectory_waypoints.clear();
    _trajectory_waypoints = {
        Eigen::Vector2f(0.0f, 0.0f),
        Eigen::Vector2f(0.0f, 30.0f), // custom route
        Eigen::Vector2f(10.0f, 30.0f),
    };
}
''')
    monkeypatch.setattr('arva_test_protocol.run_manager._git_commit', lambda root: 'test-revision')
    return root


def test_records_next_run_with_source_route_snapshots_and_mcap(tmp_path, monkeypatch):
    root = _repository(tmp_path, monkeypatch)
    monkeypatch.setitem(
        sys.modules, 'rosbag2_py', SimpleNamespace(get_registered_writers=lambda: {'mcap'})
    )
    (root / 'tests/sitl/waypoint_follow/run_001').mkdir(parents=True)
    commands = []

    class Recorder:
        def __init__(self, command):
            commands.append(command)

        def wait(self):
            return 0

    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', Recorder)
    assert waypoint_main(['--workspace-root', str(root), '--operator', 'Test operator']) == 0
    run = root / 'tests/sitl/waypoint_follow/run_002'
    config = load_yaml(run / 'config.yaml')
    assert config['test']['planned_waypoints_ned_m'] == [[0, 0], [0, 30], [10, 30]]
    assert load_yaml(root / 'px4_roscon_25/arva_test_protocol/config/waypoint_follow.yaml')[
        'test'
    ]['planned_waypoints_ned_m'] == []
    for filename in config['artifacts']['parameter_files']:
        assert (run / 'parameters' / Path(filename).name).read_bytes() == (root / filename).read_bytes()
    metadata = load_yaml(run / 'metadata.yaml')
    assert metadata['operator'] == 'Test operator'
    assert metadata['status'] == 'recording_stopped'
    assert 'outcome' not in metadata
    command = commands[0]
    assert command[command.index('--storage') + 1] == 'mcap'
    assert command[command.index('--output') + 1] == str(run / 'bag')
    assert '/arva_master/mode_state' in command
    assert '/fmu/out/vehicle_local_position' in command
    qos = load_yaml(run / 'qos_overrides.yaml')
    assert qos['/fmu/out/vehicle_local_position']['reliability'] == 'best_effort'


def test_saved_route_and_explicit_override_prepare_offline(tmp_path, monkeypatch):
    root = _repository(tmp_path, monkeypatch)
    parameters = root / 'px4_roscon_25/arva_master/cfg/mode_executor.yaml'
    parameters.write_text('/arva_master:\n  ros__parameters:\n    waypoints_ned: [0, 0, 0, 20]\n')
    monkeypatch.chdir(root)

    def unexpected_ros(*args, **kwargs):
        pytest.fail('Dry run must not require ROS or start recording')

    monkeypatch.setattr('arva_test_protocol.waypoint_run.require_mcap', unexpected_ros)
    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', unexpected_ros)
    assert waypoint_main(['--dry-run']) == 0
    assert waypoint_main(['--dry-run', '--waypoints-ned', '1', '2', '3', '4']) == 0
    for run_id, route in [('001', [[0, 0], [0, 20]]), ('002', [[1, 2], [3, 4]])]:
        run = root / f'tests/sitl/waypoint_follow/run_{run_id}'
        assert load_yaml(run / 'config.yaml')['test']['planned_waypoints_ned_m'] == route
        assert load_yaml(run / 'metadata.yaml')['status'] == 'prepared'
        assert not (run / 'bag').exists()


@pytest.mark.parametrize('values', [['0', '0', '1'], ['0', '0', 'nan', '1']])
def test_invalid_route_does_not_create_run(tmp_path, monkeypatch, values):
    root = _repository(tmp_path, monkeypatch)
    with pytest.raises(SystemExit) as error:
        waypoint_main(['--workspace-root', str(root), '--dry-run', '--waypoints-ned', *values])
    assert error.value.code == 2
    assert not (root / 'tests').exists()
