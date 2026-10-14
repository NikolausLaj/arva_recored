from pathlib import Path

import pytest

from arva_test_protocol.run_manager import (
    prepare_run,
    record_main,
    validate_recording_configuration,
)
from arva_test_protocol.yaml_utils import ProtocolError, dump_yaml, load_yaml


_CONFIG_DIRECTORY = Path(__file__).parent.parent / 'config'
_TEMPLATE_NAMES = [
    'arva_sensor_characterization',
    'heading_controller',
    'terrain_follow',
    'waypoint_follow',
    'fluxline_follow',
    'fine_search',
    'full_search',
]


@pytest.mark.parametrize('template_name', _TEMPLATE_NAMES)
def test_unfilled_templates_are_rejected_before_recording(template_name):
    config = load_yaml(_CONFIG_DIRECTORY / f'{template_name}.yaml')

    with pytest.raises(ProtocolError):
        validate_recording_configuration(config, {})


def test_prepare_run_records_manual_source_revision(tmp_path):
    config_path = tmp_path / 'config.yaml'
    dump_yaml(
        config_path,
        {
            'test_type': 'waypoint_follow',
            'recording': {'topics': ['/example/topic']},
            'artifacts': {'parameter_files': []},
            'metadata_defaults': {},
            'test': {'planned_waypoints_ned_m': [[0.0, 0.0], [1.0, 1.0]]},
            'analysis': {'kind': 'waypoint_follow'},
        },
    )

    run_directory = prepare_run(
        config_path=config_path,
        output_root=tmp_path / 'runs',
        workspace_root=tmp_path,
        software_revision='sitl-build-2026-09-27',
    )

    metadata = load_yaml(run_directory / 'metadata.yaml')
    assert metadata['software_commit'] == 'sitl-build-2026-09-27'


@pytest.mark.parametrize('storage_id', ['sqlite3', 'mcap'])
def test_recording_storage_selection_preserves_run_lifecycle(tmp_path, monkeypatch, storage_id):
    config_path = tmp_path / 'config.yaml'
    config = load_yaml(_CONFIG_DIRECTORY / 'heading_controller.yaml')
    config['artifacts']['parameter_files'] = []
    config['analysis']['segments'] = [
        {'id': 'step', 'start_event': 'step_start', 'end_event': 'step_end'}
    ]
    dump_yaml(config_path, config)
    run_directory = prepare_run(config_path, tmp_path / 'runs', tmp_path)
    commands = []

    class RecorderProcess:
        def __init__(self, command):
            commands.append(command)
            metadata = load_yaml(run_directory / 'metadata.yaml')
            assert metadata['status'] == 'recording'
            assert metadata['storage_id'] == storage_id

        def wait(self):
            return 0

    monkeypatch.setattr('arva_test_protocol.run_manager.subprocess.Popen', RecorderProcess)
    arguments = ['--run-dir', str(run_directory), '--confirm']
    if storage_id == 'mcap':
        arguments.extend(['--storage-id', 'mcap'])

    assert record_main(arguments) == 0
    command = commands[0]
    assert command[command.index('--storage') + 1] == storage_id
    assert command[command.index('--output') + 1] == str(run_directory / 'bag')
    qos_path = Path(command[command.index('--qos-profile-overrides-path') + 1])
    assert qos_path == run_directory / 'qos_overrides.yaml'
    assert load_yaml(qos_path) == {
        topic: {
            'reliability': 'best_effort', 'durability': 'volatile',
            'history': 'keep_last', 'depth': 10,
        }
        for topic in ['/fmu/out/vehicle_local_position', '/fmu/out/vehicle_attitude']
    }
    assert '/pieps/measurement' in command
    metadata = load_yaml(run_directory / 'metadata.yaml')
    assert metadata['status'] == 'recording_stopped'
    assert metadata['bag_path'] == 'bag'
