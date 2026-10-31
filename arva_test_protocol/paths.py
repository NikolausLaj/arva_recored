"""Locate templates and artifacts in workshop, standalone, and installed layouts."""

from pathlib import Path

from .yaml_utils import ProtocolError


def _package_share(package: str) -> Path | None:
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(package))
    except (ImportError, LookupError):
        return None


def workspace_root(explicit_root: str | None, template: Path) -> Path:
    """Honor an explicit root, or detect a workshop/ROS workspace near cwd."""
    if explicit_root:
        root = Path(explicit_root).expanduser().resolve()
        if not root.is_dir():
            raise ProtocolError(f'Workspace root is not a directory: {root}')
        return root
    current = Path.cwd()
    for candidate in [current, *current.parents]:
        if any((candidate / path).exists() for path in (
            template, Path('px4_roscon_25/arva_master'), Path('src/arva_master'),
            Path('arva_master'), Path('src/roscon-25-workshop'),
        )):
            workshop = candidate / 'src/roscon-25-workshop'
            if workshop.is_dir():
                return workshop
            if candidate.name in {'src', 'px4_roscon_25'}:
                return candidate.parent
            return candidate
    return current


def template_path(root: Path, template: Path, explicit: str | None = None) -> Path:
    """Use an explicit template, a workshop override, or this package's assets."""
    if explicit:
        candidate = Path(explicit).expanduser().resolve()
    elif (root / template).is_file():
        candidate = root / template
    else:
        # A source checkout should use its own templates, even if an older
        # installed version of this package is also sourced.
        source = Path(__file__).resolve().parent.parent / 'config' / template.name
        share = None if source.is_file() else _package_share('arva_test_protocol')
        candidate = source if share is None else share / 'config' / template.name
    if not candidate.is_file():
        raise ProtocolError(f'Test template does not exist: {candidate}')
    return candidate


def artifact_path(root: Path, value: str | Path) -> Path:
    """Resolve legacy workshop paths against source packages or the ROS index."""
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    direct = root / path
    if direct.is_file():
        return direct
    parts = path.parts
    if parts and parts[0] == 'px4_roscon_25':
        parts = parts[1:]
    if len(parts) >= 2:
        for base in (root, root / 'src', root / 'px4_roscon_25'):
            candidate = base.joinpath(*parts)
            if candidate.is_file():
                return candidate
        share = _package_share(parts[0])
        if share is not None:
            candidate = share.joinpath(*parts[1:])
            if candidate.is_file():
                return candidate
    return direct
