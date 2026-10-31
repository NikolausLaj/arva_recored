"""Shared options and dependency checks for quick flight-test recordings."""

from pathlib import Path

from .paths import template_path, workspace_root as workspace_root
from .yaml_utils import ProtocolError, load_yaml


def require_mcap() -> None:
    """Check MCAP support before creating a run."""
    try:
        from rosbag2_py import get_registered_writers
    except ImportError as error:
        raise ProtocolError(
            'Cannot load rosbag2_py; source your ROS workspace before recording.'
        ) from error
    if 'mcap' not in get_registered_writers():
        raise ProtocolError(
            'MCAP storage support is missing. Install ros-humble-rosbag2-storage-mcap '
            'inside this container. If sudo is unavailable, run from the Docker host: '
            'docker exec --user root <container> apt-get install -y '
            'ros-humble-rosbag2-storage-mcap'
        )


def add_run_options(parser) -> None:
    """Add options shared by SITL and physical-flight shortcuts."""
    parser.add_argument('--environment', choices=['sitl', 'real_world'], default='sitl')
    parser.add_argument('--output-root', help='Run storage directory, relative to workspace root')
    parser.add_argument('--config', help='Custom test YAML; defaults to the packaged template')


def run_configuration(root: Path, template: Path, options):
    """Load a template and select data paths without assuming a source layout."""
    source = template_path(root, template, options.config)
    config = load_yaml(source)
    config.setdefault('metadata_defaults', {})['environment'] = options.environment
    if options.environment == 'real_world':
        artifacts = config.setdefault('artifacts', {})
        artifacts['parameter_files'] = [
            value for value in artifacts.get('parameter_files', [])
            if 'arva_sim' not in Path(value).parts
        ]
    output = (
        Path(options.output_root).expanduser()
        if options.output_root else Path('tests') / options.environment
    )
    if not output.is_absolute():
        output = root / output
    return source, config, output
