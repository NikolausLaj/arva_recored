from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import pytest

from arva_test_protocol.terrain_run import terrain_main
from arva_test_protocol.yaml_utils import load_yaml


_PACKAGE = Path(__file__).parent.parent


def _repository(tmp_path, monkeypatch):
    root = tmp_path / 'repository'
    template = root / 'px4_roscon_25/arva_test_protocol/config/terrain_follow.yaml'
    template.parent.mkdir(parents=True)
    shutil.copy2(_PACKAGE / 'config/terrain_follow.yaml', template)
    parameters = root / 'px4_roscon_25/arva_master/cfg/terrain_follow.yaml'
    parameters.parent.mkdir(parents=True)
    parameters.write_text(
        'terrain_follow_controller:\n'
        '  ros__parameters:\n'
        '    target_distance: 2.0 # saved target\n'
        '    kp: 2.75\n'
    )
    monkeypatch.setattr('arva_test_protocol.run_manager._git_commit', lambda root: 'test-revision')
    return root, parameters


def test_default_records_mcap_and_versions_exact_yaml_snapshots(tmp_path, monkeypatch):
    root, parameters = _repository(tmp_path, monkeypatch)
    monkeypatch.setitem(
        sys.modules, 'rosbag2_py', SimpleNamespace(get_registered_writers=lambda: {'mcap'})
    )

    def unexpected_query():
        pytest.fail('Default recording must not query a live controller')

    monkeypatch.setattr('arva_test_protocol.terrain_run._live_parameters', unexpected_query)
    test_directory = root / 'tests/sitl/terrain_follow'
    (test_directory / 'run_001').mkdir(parents=True)
    commands = []

    class Recorder:
        def __init__(self, command):
            commands.append(command)

        def wait(self):
            return 0

    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', Recorder)
    original_parameters = parameters.read_bytes()
    for run_id, target in [('002', 2.0), ('003', 3.0)]:
        assert terrain_main(['--workspace-root', str(root), '--operator', 'Test operator']) == 0
        run_directory = test_directory / f'run_{run_id}'
        assert (run_directory / 'parameters/terrain_follow.yaml').read_bytes() == parameters.read_bytes()
        config = load_yaml(run_directory / 'config.yaml')
        assert config['analysis']['segments'] == [{
            'id': 'terrain_tracking', 'start_offset_s': 0.0,
            'end_of_bag': True, 'commanded_agl_m': target,
        }]
        assert config['test']['step_targets_agl_m'] == []
        metadata = load_yaml(run_directory / 'metadata.yaml')
        assert metadata['status'] == 'recording_stopped'
        assert metadata['operator'] == 'Test operator'
        assert metadata['parameter_snapshots'] == ['parameters/terrain_follow.yaml']
        assert 'outcome' not in metadata
        command = commands[-1]
        assert command[:3] == ['ros2', 'bag', 'record']
        assert command[command.index('--storage') + 1] == 'mcap'
        assert command[command.index('--output') + 1] == str(run_directory / 'bag')
        qos = load_yaml(Path(command[command.index('--qos-profile-overrides-path') + 1]))
        assert set(qos) == {'/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude'}
        assert all(profile['reliability'] == 'best_effort' for profile in qos.values())
        assert '/fmu/out/vehicle_local_position' in command
        assert '/terrain_controller/cmd_vel' in command
        parameters.write_bytes(original_parameters.replace(b'2.0', b'3.0'))

    assert len(commands) == 2
    assert (test_directory / 'run_002/parameters/terrain_follow.yaml').read_bytes() == original_parameters


def test_dry_run_prepares_without_ros_or_recording(tmp_path, monkeypatch):
    root, parameters = _repository(tmp_path, monkeypatch)
    monkeypatch.chdir(root)

    def unexpected_ros(*args, **kwargs):
        pytest.fail('Dry run must not require ROS or start a recorder')

    monkeypatch.setattr('arva_test_protocol.terrain_run.require_mcap', unexpected_ros)
    monkeypatch.setattr('arva_test_protocol.terrain_run._live_parameters', unexpected_ros)
    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', unexpected_ros)
    assert terrain_main(['--dry-run']) == 0
    run_directory = root / 'tests/sitl/terrain_follow/run_001'
    assert load_yaml(run_directory / 'metadata.yaml')['status'] == 'prepared'
    assert (run_directory / 'parameters/terrain_follow.yaml').read_bytes() == parameters.read_bytes()
    assert not (run_directory / 'bag').exists()


def test_live_parameters_are_opt_in(tmp_path, monkeypatch):
    root, _ = _repository(tmp_path, monkeypatch)
    runtime_parameters = {'/terrain_follow_controller': {'ros__parameters': {'target_distance': 1.5}}}
    monkeypatch.setattr(
        'arva_test_protocol.terrain_run._live_parameters', lambda: runtime_parameters
    )
    assert terrain_main(['--workspace-root', str(root), '--live-parameters', '--dry-run']) == 0
    run_directory = root / 'tests/sitl/terrain_follow/run_001'
    assert load_yaml(run_directory / 'parameters/terrain_follow_runtime.yaml') == runtime_parameters
    config = load_yaml(run_directory / 'config.yaml')
    assert config['analysis']['segments'][0]['commanded_agl_m'] == 1.5
