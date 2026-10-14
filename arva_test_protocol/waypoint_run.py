"""Prepare and record a single SITL waypoint-follow trial with one command."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shlex
from typing import Sequence

from .quick_run import require_mcap, workspace_root
from .run_manager import prepare_run, record_run, validate_recording_configuration
from .trajectory import parse_xy_waypoints
from .yaml_utils import ProtocolError, dump_yaml, load_yaml


_TEMPLATE = Path('px4_roscon_25/arva_test_protocol/config/waypoint_follow.yaml')
_PARAMETERS = Path('px4_roscon_25/arva_master/cfg/mode_executor.yaml')
_SOURCE = Path('px4_roscon_25/arva_master/src/WaypointFollowMode.cpp')


def _saved_waypoints(root: Path) -> tuple[list[list[float]], str]:
    parameters = load_yaml(root / _PARAMETERS)
    node = parameters.get('/arva_master', parameters.get('arva_master', {}))
    if not isinstance(node, dict) or not isinstance(node.get('ros__parameters'), dict):
        raise ProtocolError('Mode executor parameters must contain /arva_master.ros__parameters')
    configured = node['ros__parameters'].get('waypoints_ned', [])
    if configured != []:
        points = parse_xy_waypoints(configured, 'waypoints_ned')
        return [list(point) for point in points], 'saved mode_executor.yaml waypoints_ned'

    # Read the actual fallback route instead of maintaining a second route here.
    try:
        source = (root / _SOURCE).read_text(encoding='utf-8')
    except OSError as error:
        raise ProtocolError(f'Cannot read fallback waypoints from {root / _SOURCE}') from error
    source = re.sub(r'/\*.*?\*/|//[^\n]*', '', source, flags=re.DOTALL)
    body = re.search(
        r'void\s+WaypointFollow::initialiseWaypoints\s*\(\s*\).*?'
        r'_trajectory_waypoints\s*=\s*\{(.*?)\};', source, flags=re.DOTALL,
    )
    if body is None:
        raise ProtocolError('Cannot read fallback route; supply --waypoints-ned explicitly')
    number = r'([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)[fF]?'
    pattern = rf'Eigen::Vector2f\s*\(\s*{number}\s*,\s*{number}\s*\)\s*,?'
    if re.sub(pattern, '', body.group(1)).strip():
        raise ProtocolError('Cannot parse fallback route; supply --waypoints-ned explicitly')
    pairs = [[float(north), float(east)] for north, east in re.findall(pattern, body.group(1))]
    points = parse_xy_waypoints(pairs, 'WaypointFollowMode.cpp fallback route')
    return [list(point) for point in points], 'WaypointFollowMode.cpp fallback route'


def waypoint_main(arguments: Sequence[str] | None = None) -> int:
    """Prepare the next waypoint run and record MCAP until Ctrl-C."""
    parser = argparse.ArgumentParser(
        description='Record one SITL waypoint-follow trial to MCAP; stop with Ctrl-C.'
    )
    parser.add_argument('--workspace-root', help='Repository root; detected by default')
    parser.add_argument('--run-id', help='Run ID; defaults to the next available number')
    parser.add_argument('--operator', help='Operator name saved with the run')
    parser.add_argument(
        '--waypoints-ned', nargs='+', type=float, metavar='COORDINATE',
        help='Analysis route as flattened north/east pairs, for runtime overrides. '
             'Does not change the vehicle route.',
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
        if options.waypoints_ned is None:
            waypoints, route_source = _saved_waypoints(root)
        else:
            waypoints = [list(point) for point in parse_xy_waypoints(
                options.waypoints_ned, '--waypoints-ned'
            )]
            route_source = '--waypoints-ned (analysis reference only)'
        config = load_yaml(root / _TEMPLATE)
        config['test']['planned_waypoints_ned_m'] = waypoints
        validate_recording_configuration(config, {})
        run_directory = prepare_run(
            config_path=root / _TEMPLATE,
            output_root=root / 'tests/sitl',
            workspace_root=root,
            run_id=options.run_id,
            operator=options.operator,
            notes=f'Single waypoint-follow trial; planned route from {route_source}.',
        )
        dump_yaml(run_directory / 'config.yaml', config)
        print(f'[arva test protocol] run: {run_directory}', flush=True)
        print(f'[arva test protocol] planned NED route: {waypoints} ({route_source})', flush=True)
        print(
            '[arva test protocol] Start the waypoint trial after recording begins; '
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
