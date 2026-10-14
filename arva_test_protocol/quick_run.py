"""Shared dependency and repository checks for quick SITL recordings."""

from pathlib import Path

from .yaml_utils import ProtocolError


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


def workspace_root(explicit_root: str | None, template: Path) -> Path:
    """Find the repository containing the selected test template."""
    if explicit_root:
        candidates = [Path(explicit_root).expanduser().resolve()]
    else:
        current = Path.cwd()
        candidates = [current, *current.parents]
        candidates.extend([
            current / 'src/roscon-25-workshop',
            Path.home() / 'roscon-25-workshop_ws/src/roscon-25-workshop',
        ])
    for candidate in candidates:
        if (candidate / template).is_file():
            return candidate
    raise ProtocolError(
        'Cannot find the repository; pass --workspace-root /path/to/roscon-25-workshop'
    )
