"""Exercise physical-flight runs without a workshop or simulator checkout."""

from pathlib import Path
import shutil

import pytest

from arva_test_protocol import paths
from arva_test_protocol.fluxline_run import fluxline_main
from arva_test_protocol.heading_run import heading_main
from arva_test_protocol.waypoint_run import waypoint_main
from arva_test_protocol.yaml_utils import load_yaml


def _drone_workspace(tmp_path):
    root = tmp_path / 'drone_ws'
    config = root / 'src/arva_master/cfg'
    config.mkdir(parents=True)
    for name in ['waypoint_follow', 'terrain_follow', 'fluxline_follow']:
        (config / f'{name}.yaml').write_text('example_parameter: 1\n')
    (config / 'mode_executor.yaml').write_text(
        '/arva_master:\n  ros__parameters:\n    waypoints_ned: []\n'
    )
    return root


def test_physical_fluxline_snapshots_source_packages_without_simulator(tmp_path):
    root = _drone_workspace(tmp_path)
    output = tmp_path / 'flight_data'
    assert fluxline_main([
        '--workspace-root', str(root), '--environment', 'real_world',
        '--output-root', str(output), '--dry-run', '--transmitter-ned', '12', '-3', '0',
    ]) == 0
    run = output / 'fluxline_follow/run_001'
    config = load_yaml(run / 'config.yaml')
    assert config['test']['transmitter_position_ned_m'] == [12, -3, 0]
    assert len(list((run / 'parameters').glob('*.yaml'))) == 3
    assert load_yaml(run / 'metadata.yaml')['environment'] == 'real_world'
    assert not (run / 'bag').exists()


def test_physical_fluxline_requires_surveyed_position_even_with_simulator(tmp_path):
    root = _drone_workspace(tmp_path)
    simulator = root / 'src/arva_sim/cfg/arva_sim.yaml'
    simulator.parent.mkdir(parents=True)
    simulator.write_text(
        'arva_sim_node:\n  ros__parameters:\n'
        '    transmitter_position_ned_m: [30, 20, 0]\n    transmitter_heading_deg: 0\n'
    )
    with pytest.raises(SystemExit):
        fluxline_main([
            '--workspace-root', str(root), '--environment', 'real_world', '--dry-run',
        ])
    assert not (root / 'tests').exists()


def test_physical_waypoints_require_explicit_route_and_detect_ros_workspace(tmp_path, monkeypatch):
    root = _drone_workspace(tmp_path)
    monkeypatch.chdir(root / 'src')
    with pytest.raises(SystemExit):
        waypoint_main(['--environment', 'real_world', '--dry-run'])
    assert not (root / 'tests').exists()
    assert waypoint_main([
        '--environment', 'real_world', '--dry-run', '--waypoints-ned', '0', '0', '0', '10',
    ]) == 0
    run = root / 'tests/real_world/waypoint_follow/run_001'
    assert load_yaml(run / 'config.yaml')['test']['planned_waypoints_ned_m'] == [[0, 0], [0, 10]]
    assert len(list((run / 'parameters').glob('*.yaml'))) == 3


def test_installed_templates_and_controller_artifacts_work_without_source(tmp_path, monkeypatch):
    root = tmp_path / 'empty_workspace'
    root.mkdir()
    protocol_share = tmp_path / 'install/share/arva_test_protocol'
    (protocol_share / 'config').mkdir(parents=True)
    shutil.copy2(
        Path(__file__).parent.parent / 'config/heading_controller.yaml',
        protocol_share / 'config/heading_controller.yaml',
    )
    master_share = tmp_path / 'install/share/arva_master'
    (master_share / 'cfg').mkdir(parents=True)
    for name in ['fluxline_follow', 'mode_executor']:
        (master_share / f'cfg/{name}.yaml').write_text('example_parameter: 1\n')
    shares = {'arva_test_protocol': protocol_share, 'arva_master': master_share}
    monkeypatch.setattr(paths, '_package_share', shares.get)
    monkeypatch.setattr(paths, '__file__', str(tmp_path / 'site-packages/arva_test_protocol/paths.py'))
    assert heading_main([
        '--workspace-root', str(root), '--environment', 'real_world', '--dry-run',
    ]) == 0
    run = root / 'tests/real_world/heading_controller/run_001'
    assert len(list((run / 'parameters').glob('*.yaml'))) == 2
    assert load_yaml(run / 'metadata.yaml')['configuration_source'] == str(
        protocol_share / 'config/heading_controller.yaml'
    )


def test_custom_template_and_absolute_artifacts_from_another_directory(tmp_path, monkeypatch):
    root = _drone_workspace(tmp_path)
    custom = tmp_path / 'custom_waypoints.yaml'
    custom.write_text(
        'test_type: waypoint_follow\nrecording:\n  topics: [/example/topic]\n'
        f'artifacts:\n  parameter_files: [{root / "src/arva_master/cfg/waypoint_follow.yaml"}]\n'
        'test:\n  planned_waypoints_ned_m: [[1, 2], [3, 4]]\n'
        'analysis:\n  kind: waypoint_follow\n'
    )
    monkeypatch.chdir(tmp_path)
    assert waypoint_main([
        '--workspace-root', str(root), '--config', str(custom),
        '--environment', 'real_world', '--output-root', 'flight_data', '--dry-run',
    ]) == 0
    run = root / 'flight_data/waypoint_follow/run_001'
    assert load_yaml(run / 'config.yaml')['test']['planned_waypoints_ned_m'] == [[1, 2], [3, 4]]
    assert (run / 'parameters/waypoint_follow.yaml').is_file()
