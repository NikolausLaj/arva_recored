from pathlib import Path
import shutil
import sys
from types import SimpleNamespace

import pytest

from arva_test_protocol.heading_run import heading_main
from arva_test_protocol.run_manager import validate_recording_configuration
from arva_test_protocol.yaml_utils import ProtocolError, load_yaml


_PACKAGE = Path(__file__).parent.parent


def _repository(tmp_path):
    root = tmp_path / 'repository'
    template = root / 'px4_roscon_25/arva_test_protocol/config/heading_controller.yaml'
    template.parent.mkdir(parents=True)
    shutil.copy2(_PACKAGE / 'config/heading_controller.yaml', template)
    config = load_yaml(template)
    for filename in config['artifacts']['parameter_files']:
        parameter_file = root / filename
        parameter_file.parent.mkdir(parents=True, exist_ok=True)
        parameter_file.write_text('example_parameter: 1\n')
    return root, template


def test_shortcut_numbers_runs_snapshots_parameters_and_records_mcap(tmp_path, monkeypatch):
    root, template = _repository(tmp_path)
    monkeypatch.setitem(
        sys.modules, 'rosbag2_py', SimpleNamespace(get_registered_writers=lambda: {'mcap'})
    )
    monkeypatch.setattr('arva_test_protocol.run_manager._git_commit', lambda root: 'test-revision')
    test_directory = root / 'tests/sitl/heading_controller'
    (test_directory / 'run_001').mkdir(parents=True)
    run_directory = test_directory / 'run_002'
    commands = []

    class Recorder:
        def __init__(self, command):
            commands.append(command)

        def wait(self):
            return 0

    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', Recorder)
    assert heading_main(['--workspace-root', str(root), '--operator', 'Test operator']) == 0
    config = load_yaml(run_directory / 'config.yaml')
    validate_recording_configuration(config, {})
    assert config['analysis']['use_active_mode_interval'] is True
    assert config['analysis']['segments'] == []
    assert config['test']['heading_offsets_deg'] == []
    assert len(list((run_directory / 'parameters').glob('*.yaml'))) == 2
    assert 'use_active_mode_interval' not in load_yaml(template)['analysis']
    metadata = load_yaml(run_directory / 'metadata.yaml')
    assert metadata['status'] == 'recording_stopped'
    assert 'outcome' not in metadata
    assert metadata['operator'] == 'Test operator'
    command = commands[0]
    assert command[command.index('--storage') + 1] == 'mcap'
    assert command[command.index('--output') + 1] == str(run_directory / 'bag')
    assert '/pieps/measurement' in command


def test_dry_run_detects_repository_and_never_starts_recorder(tmp_path, monkeypatch):
    root, _ = _repository(tmp_path)
    monkeypatch.setattr('arva_test_protocol.run_manager._git_commit', lambda root: 'test-revision')
    monkeypatch.chdir(root)

    def unexpected_recording(command):
        pytest.fail('Dry run must not start a recorder')

    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', unexpected_recording)
    assert heading_main(['--dry-run']) == 0
    run_directory = root / 'tests/sitl/heading_controller/run_001'
    assert load_yaml(run_directory / 'metadata.yaml')['status'] == 'prepared'


def test_active_interval_does_not_bypass_invalid_explicit_segments():
    config = {
        'analysis': {
            'kind': 'heading_controller',
            'use_active_mode_interval': True,
            'segments': [{'id': 'missing_boundaries'}],
        }
    }
    with pytest.raises(ProtocolError):
        validate_recording_configuration(config, {})


def test_missing_mcap_fails_before_creating_run(tmp_path, monkeypatch, capsys):
    root, _ = _repository(tmp_path)
    monkeypatch.setitem(
        sys.modules, 'rosbag2_py', SimpleNamespace(get_registered_writers=lambda: {'sqlite3'})
    )
    with pytest.raises(SystemExit) as error:
        heading_main(['--workspace-root', str(root)])
    assert error.value.code == 2
    assert 'ros-humble-rosbag2-storage-mcap' in capsys.readouterr().err
    assert not (root / 'tests').exists()
