"""Prepare and record a constant-target SITL terrain trial with one command."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shlex
import subprocess
from typing import Any, Sequence

import yaml

from .quick_run import require_mcap, workspace_root
from .run_manager import prepare_run, record_run, validate_recording_configuration
from .yaml_utils import ProtocolError, dump_yaml, load_yaml


_TEMPLATE = Path('px4_roscon_25/arva_test_protocol/config/terrain_follow.yaml')
_PARAMETERS = Path('px4_roscon_25/arva_master/cfg/terrain_follow.yaml')
_NODE = '/terrain_follow_controller'


def _live_parameters() -> dict[str, Any]:
    try:
        result = subprocess.run(
            ['ros2', 'param', 'dump', _NODE],
            check=True, capture_output=True, text=True, timeout=10,
        )
        parameters = yaml.safe_load(result.stdout)
        _target_from_parameters(parameters)
    except (OSError, subprocess.SubprocessError, yaml.YAMLError, ProtocolError) as error:
        raise ProtocolError(
            f'Cannot read {_NODE} parameters. Start the ARVA launch first, or omit '
            '--live-parameters to use the saved controller YAML.'
        ) from error
    return parameters


def _target_from_parameters(parameters: Any) -> float:
    if not isinstance(parameters, dict):
        raise ProtocolError('Terrain parameters must be a YAML mapping')
    node_parameters = parameters.get(_NODE, parameters.get(_NODE.lstrip('/')))
    if not isinstance(node_parameters, dict):
        raise ProtocolError(f'Terrain parameters must contain {_NODE}')
    values = node_parameters.get('ros__parameters')
    if not isinstance(values, dict):
        raise ProtocolError('Terrain parameters must contain ros__parameters')
    return _positive_target(values.get('target_distance'))


def _positive_target(value: Any) -> float:
    if (
        not isinstance(value, (int, float)) or isinstance(value, bool)
        or not math.isfinite(value) or value <= 0.0
    ):
        raise ProtocolError('Terrain target must be a finite value greater than zero')
    return float(value)


def terrain_main(arguments: Sequence[str] | None = None) -> int:
    """Prepare the next terrain run and record MCAP until Ctrl-C."""
    parser = argparse.ArgumentParser(
        description='Record a constant-target SITL terrain trial to MCAP; stop with Ctrl-C.'
    )
    parser.add_argument('--workspace-root', help='Repository root; detected by default')
    parser.add_argument('--run-id', help='Run ID; defaults to the next available number')
    parser.add_argument('--operator', help='Operator name saved with the run')
    target_options = parser.add_mutually_exclusive_group()
    target_options.add_argument(
        '--target-agl', type=float,
        help='Analysis target in meters; defaults to target_distance in the controller YAML. '
             'Does not change the controller target.',
    )
    target_options.add_argument(
        '--live-parameters', action='store_true',
        help='Read the analysis target from the running controller and save its parameter dump',
    )
    parser.add_argument(
        '--dry-run', action='store_true',
        help='Prepare a run and print its command without starting recording',
    )
    options = parser.parse_args(arguments)
    try:
        root = workspace_root(options.workspace_root, _TEMPLATE)
        if not options.dry_run:
            require_mcap()
        runtime_parameters = None
        if options.target_agl is not None:
            target = _positive_target(options.target_agl)
            target_source = '--target-agl (analysis reference only)'
        elif options.live_parameters:
            runtime_parameters = _live_parameters()
            target = _target_from_parameters(runtime_parameters)
            target_source = f'{_NODE}.target_distance at recording start'
        else:
            target = _target_from_parameters(load_yaml(root / _PARAMETERS))
            target_source = 'source controller YAML'

        config = load_yaml(root / _TEMPLATE)
        config['analysis']['segments'] = [{
            'id': 'terrain_tracking',
            'start_offset_s': 0.0,
            'end_of_bag': True,
            'commanded_agl_m': target,
        }]
        config['test']['step_targets_agl_m'] = []
        config['test']['terrain_tracking_target_agl_m'] = target
        config['test']['repetitions'] = 1
        validate_recording_configuration(config, {})
        run_directory = prepare_run(
            config_path=root / _TEMPLATE,
            output_root=root / 'tests/sitl',
            workspace_root=root,
            run_id=options.run_id,
            operator=options.operator,
            notes=f'Constant-target terrain trial; target from {target_source}.',
        )
        dump_yaml(run_directory / 'config.yaml', config)
        if runtime_parameters is not None:
            dump_yaml(
                run_directory / 'parameters/terrain_follow_runtime.yaml', runtime_parameters
            )
        print(f'[arva test protocol] run: {run_directory}', flush=True)
        print(f'[arva test protocol] analysis target: {target:g} m ({target_source})', flush=True)
        print(
            '[arva test protocol] Keep the terrain target constant during this recording; '
            'stop with Ctrl-C. No event markers are required.', flush=True,
        )
        result = record_run(
            run_directory, confirmed=True, storage_id='mcap', dry_run=options.dry_run
        )
        if result == 0 and not options.dry_run:
            print(f'[arva test protocol] MCAP directory: {run_directory / "bag"}')
            print('Analyze this trial with:')
            print(
                'ros2 run arva_test_protocol arva_test_analyze '
                f'--run-dir {shlex.quote(str(run_directory))}'
            )
            print('Declare the trial outcome separately with arva_test_finish.')
        return result
    except ProtocolError as error:
        parser.error(str(error))
    return 1
