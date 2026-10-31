"""Prepare and record a single SITL fluxline-follow trial with one command."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import shlex
from typing import Any, Sequence

from .quick_run import require_mcap, workspace_root
from .run_manager import prepare_run, record_run, validate_recording_configuration
from .yaml_utils import ProtocolError, dump_yaml, load_yaml


_TEMPLATE = Path('px4_roscon_25/arva_test_protocol/config/fluxline_follow.yaml')
_SIMULATOR_CONFIG = Path('px4_roscon_25/arva_sim/cfg/arva_sim.yaml')


def _position(value: Any) -> list[float]:
    if not isinstance(value, list) or len(value) != 3 or not all(
        isinstance(item, (int, float)) and not isinstance(item, bool) and math.isfinite(item)
        for item in value
    ):
        raise ProtocolError('Transmitter position must contain three finite NED coordinates')
    return [float(item) for item in value]


def _simulator_transmitter(root: Path) -> tuple[list[float], float]:
    """Read the transmitter analysis reference from the saved simulator YAML."""
    config = load_yaml(root / _SIMULATOR_CONFIG)
    node = config.get('/arva_sim_node', config.get('arva_sim_node'))
    if not isinstance(node, dict) or not isinstance(node.get('ros__parameters'), dict):
        raise ProtocolError('Simulator YAML must contain arva_sim_node.ros__parameters')
    parameters = node['ros__parameters']
    position = _position(parameters.get('transmitter_position_ned_m'))
    heading = parameters.get('transmitter_heading_deg')
    if (
        not isinstance(heading, (int, float)) or isinstance(heading, bool)
        or not math.isfinite(heading)
    ):
        raise ProtocolError('Simulator transmitter_heading_deg must be finite')
    return position, math.radians(heading)


def fluxline_main(arguments: Sequence[str] | None = None) -> int:
    """Prepare the next fluxline run and record MCAP until Ctrl-C."""
    parser = argparse.ArgumentParser(
        description='Record one SITL fluxline-follow trial to MCAP; stop with Ctrl-C.'
    )
    parser.add_argument('--workspace-root', help='Repository root; detected by default')
    parser.add_argument('--run-id', help='Run ID; defaults to the next available number')
    parser.add_argument('--operator', help='Operator name saved with the run')
    parser.add_argument(
        '--transmitter-ned', nargs=3, type=float, metavar=('NORTH', 'EAST', 'DOWN'),
        help='Transmitter analysis reference in meters; defaults to the test configuration '
             'or simulator YAML. Does not change the simulator.',
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
        config = load_yaml(root / _TEMPLATE)
        if options.transmitter_ned is not None:
            position = _position(options.transmitter_ned)
            position_source = '--transmitter-ned (analysis reference only)'
        elif config['test'].get('transmitter_position_ned_m', []) != []:
            position = _position(config['test']['transmitter_position_ned_m'])
            position_source = 'test configuration'
        else:
            position, heading = _simulator_transmitter(root)
            position_source = 'saved arva_sim.yaml'
            if config['test'].get('transmitter_heading_rad') is None:
                config['test']['transmitter_heading_rad'] = heading
        config['test']['transmitter_position_ned_m'] = position
        config['test']['start_conditions'] = []
        config['test']['repetitions_per_start_condition'] = 1
        validate_recording_configuration(config, {})
        run_directory = prepare_run(
            config_path=root / _TEMPLATE,
            output_root=root / 'tests/sitl',
            workspace_root=root,
            run_id=options.run_id,
            operator=options.operator,
            notes=f'Single fluxline-follow trial; transmitter position from {position_source}.',
        )
        dump_yaml(run_directory / 'config.yaml', config)
        print(f'[arva test protocol] run: {run_directory}', flush=True)
        print(
            f'[arva test protocol] transmitter NED: {position} ({position_source})', flush=True,
        )
        print(
            '[arva test protocol] Start the fluxline trial after recording begins; '
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
