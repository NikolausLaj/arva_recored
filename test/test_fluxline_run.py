from pathlib import Path
import math
import shutil
import signal
import sys
from types import SimpleNamespace

import pytest

from arva_test_protocol.fluxline_run import fluxline_main
from arva_test_protocol.yaml_utils import dump_yaml, load_yaml


_PACKAGE = Path(__file__).parent.parent


def _repository(tmp_path, monkeypatch):
    root = tmp_path / 'repository'
    template = root / 'px4_roscon_25/arva_test_protocol/config/fluxline_follow.yaml'
    template.parent.mkdir(parents=True)
    shutil.copy2(_PACKAGE / 'config/fluxline_follow.yaml', template)
    for filename in load_yaml(template)['artifacts']['parameter_files']:
        parameter = root / filename
        parameter.parent.mkdir(parents=True, exist_ok=True)
        parameter.write_text('example_parameter: 1\n')
    simulator = root / 'px4_roscon_25/arva_sim/cfg/arva_sim.yaml'
    dump_yaml(simulator, {'arva_sim_node': {'ros__parameters': {
        'transmitter_position_ned_m': [30.0, -20.0, 0.0],
        'transmitter_heading_deg': 45.0,
    }}})
    monkeypatch.setattr('arva_test_protocol.run_manager._git_commit', lambda root: 'test-revision')
    return root, template, simulator


def test_shortcut_numbers_runs_and_stops_mcap_with_ctrl_c(tmp_path, monkeypatch):
    root, template, _ = _repository(tmp_path, monkeypatch)
    monkeypatch.setitem(
        sys.modules, 'rosbag2_py', SimpleNamespace(get_registered_writers=lambda: {'mcap'})
    )
    commands, signals = [], []

    class Recorder:
        def __init__(self, command):
            commands.append(command)
            self.interrupted = False

        def wait(self):
            if not self.interrupted:
                self.interrupted = True
                raise KeyboardInterrupt
            return 0

        def poll(self):
            return None

        def send_signal(self, value):
            signals.append(value)

    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', Recorder)
    for run_id in ['001', '002']:
        assert fluxline_main(['--workspace-root', str(root), '--operator', 'Test operator']) == 0
        run = root / f'tests/sitl/fluxline_follow/run_{run_id}'
        config = load_yaml(run / 'config.yaml')
        assert config['test']['transmitter_position_ned_m'] == [30, -20, 0]
        assert config['test']['transmitter_heading_rad'] == pytest.approx(math.pi / 4)
        assert config['test']['repetitions_per_start_condition'] == 1
        for filename in config['artifacts']['parameter_files']:
            assert (run / 'parameters' / Path(filename).name).read_bytes() == (root / filename).read_bytes()
        metadata = load_yaml(run / 'metadata.yaml')
        assert metadata['status'] == 'recording_stopped'
        assert metadata['operator'] == 'Test operator'
        assert metadata['bag_path'] == 'bag'
        assert metadata['storage_id'] == 'mcap'
        assert 'recording_stopped_at' in metadata
        assert 'outcome' not in metadata
        command = commands[-1]
        assert command[command.index('--storage') + 1] == 'mcap'
        assert command[command.index('--output') + 1] == str(run / 'bag')
        assert set(load_yaml(template)['recording']['topics']).issubset(command)
        qos = load_yaml(run / 'qos_overrides.yaml')
        assert qos['/fmu/out/vehicle_local_position']['reliability'] == 'best_effort'
        assert '/arva_master/mode_state' in command
        assert qos['/arva_master/mode_state'] == {
            'reliability': 'reliable', 'durability': 'transient_local',
            'history': 'keep_last', 'depth': 10,
        }
    assert signals == [signal.SIGINT, signal.SIGINT]
    assert load_yaml(template)['test']['transmitter_position_ned_m'] == []


def test_offline_preview_uses_config_or_explicit_reference(tmp_path, monkeypatch):
    root, template, _ = _repository(tmp_path, monkeypatch)
    config = load_yaml(template)
    config['test']['transmitter_position_ned_m'] = [10, 20, 0]
    dump_yaml(template, config)
    monkeypatch.chdir(root)

    def unexpected_ros(*args, **kwargs):
        pytest.fail('Dry run must not require ROS or start recording')

    monkeypatch.setattr('arva_test_protocol.fluxline_run.require_mcap', unexpected_ros)
    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', unexpected_ros)
    assert fluxline_main(['--dry-run']) == 0
    assert fluxline_main(['--dry-run', '--transmitter-ned', '40', '50', '0']) == 0
    for run_id, position in [('001', [10, 20, 0]), ('002', [40, 50, 0])]:
        run = root / f'tests/sitl/fluxline_follow/run_{run_id}'
        assert load_yaml(run / 'config.yaml')['test']['transmitter_position_ned_m'] == position
        assert load_yaml(run / 'metadata.yaml')['status'] == 'prepared'
        assert not (run / 'bag').exists()


@pytest.mark.parametrize('position', [[float('nan'), 0, 0], [True, 0, 0], [0, 0]])
def test_unknown_simulator_position_fails_before_creating_run(tmp_path, monkeypatch, position):
    root, _, simulator = _repository(tmp_path, monkeypatch)
    config = load_yaml(simulator)
    config['arva_sim_node']['ros__parameters']['transmitter_position_ned_m'] = position
    dump_yaml(simulator, config)
    with pytest.raises(SystemExit) as error:
        fluxline_main(['--workspace-root', str(root), '--dry-run'])
    assert error.value.code == 2
    assert not (root / 'tests').exists()


def test_missing_mcap_fails_before_creating_run(tmp_path, monkeypatch):
    root, _, _ = _repository(tmp_path, monkeypatch)
    monkeypatch.setitem(
        sys.modules, 'rosbag2_py', SimpleNamespace(get_registered_writers=lambda: {'sqlite3'})
    )
    with pytest.raises(SystemExit) as error:
        fluxline_main(['--workspace-root', str(root)])
    assert error.value.code == 2
    assert not (root / 'tests').exists()
