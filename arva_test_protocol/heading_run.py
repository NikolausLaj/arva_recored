"""Prepare and record a single SITL heading response with one command."""

from __future__ import annotations

import argparse
from pathlib import Path
import shlex
from typing import Sequence

from .quick_run import require_mcap, workspace_root
from .run_manager import prepare_run, record_run
from .yaml_utils import ProtocolError, dump_yaml, load_yaml


_TEMPLATE = Path('px4_roscon_25/arva_test_protocol/config/heading_controller.yaml')


def heading_main(arguments: Sequence[str] | None = None) -> int:
    """Prepare the next SITL heading run and record MCAP until Ctrl-C."""
    parser = argparse.ArgumentParser(
        description='Record one SITL heading response to MCAP; stop with Ctrl-C.'
    )
    parser.add_argument('--workspace-root', help='Repository root; detected by default')
    parser.add_argument('--run-id', help='Run ID; defaults to the next available number')
    parser.add_argument('--operator', help='Operator name saved with the run')
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
        # A quick run contains one response, without manual step boundaries.
        config['analysis']['segments'] = []
        config['analysis']['use_active_mode_interval'] = True
        # Do not claim the multi-step template experiment was performed.
        config['test']['heading_offsets_deg'] = []
        config['test']['repetitions_per_offset'] = 1
        run_directory = prepare_run(
            config_path=root / _TEMPLATE,
            output_root=root / 'tests/sitl',
            workspace_root=root,
            run_id=options.run_id,
            operator=options.operator,
            notes='Single heading response; analyze the fluxline_follow interval.',
        )
        dump_yaml(run_directory / 'config.yaml', config)
        print(f'[arva test protocol] run: {run_directory}', flush=True)
        print(
            '[arva test protocol] Start the heading trial after recording begins; '
            'stop with Ctrl-C. No event markers are required.',
            flush=True,
        )
        result = record_run(
            run_directory, confirmed=True, storage_id='mcap', dry_run=options.dry_run
        )
        if result == 0 and not options.dry_run:
            quoted_directory = shlex.quote(str(run_directory))
            print(f'[arva test protocol] MCAP directory: {run_directory / "bag"}')
            print('Analyze this response with:')
            print(
                'ros2 run arva_test_protocol arva_test_analyze '
                f'--run-dir {quoted_directory}'
            )
            print('Declare the trial outcome separately with arva_test_finish.')
        return result
    except ProtocolError as error:
        parser.error(str(error))
    return 1
