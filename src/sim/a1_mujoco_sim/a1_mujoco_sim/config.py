"""Configuration loading and path resolution for the A1 MuJoCo simulator."""
from __future__ import annotations

import dataclasses
import pathlib

import yaml
from ament_index_python.packages import get_package_share_directory

# These names are part of the A1 model/ROS contract, rather than runtime
# configuration. Keep them here so callers do not duplicate string literals.
DEPTH_CAMERA_NAME = 'depth_camera'
RESET_KEYFRAME = 'home'


@dataclasses.dataclass(frozen=True)
class SimConfig:
    """Validated simulator configuration loaded from ``sim.yaml``."""

    model_path: pathlib.Path
    #Frequency
    sim_frequency: float = 500.0
    state_publish_frequency: float = 100.0
    depth_publish_frequency: float = 20.0
    imu_publish_frequency: float = 200.0
    #topic names
    command_topic: str = '/a1/command'
    state_topic: str = '/a1/state'
    depth_topic: str = '/a1/depth'
    imu_topic: str = '/a1/imu'
    reset_service: str = '/a1/reset'
    pause_service: str = '/a1/pause'
    resume_service: str = '/a1/resume'
    #depth camera
    depth_width: int = 106
    depth_height: int = 60
    command_timeout: float = 0.25

    @property
    def timestep(self) -> float:
        """Physics timestep derived from the configured simulation frequency."""
        return 1.0 / self.sim_frequency


def default_config_path() -> pathlib.Path:
    """Return the shipped ``config/sim.yaml``.

    Checked in order:
    1. next to the Python package (working tree, or ``--symlink-install``),
    2. the package share directory (regular ``colcon build`` installs
       ``data_files`` there).
    """
    candidates = [
        pathlib.Path(__file__).resolve().parent.parent / 'config' / 'sim.yaml',
        pathlib.Path(get_package_share_directory('a1_mujoco_sim')) / 'config' / 'sim.yaml',
    ]
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    raise FileNotFoundError(
        'Cannot find simulator configuration; looked in: '
        + ', '.join(str(candidate) for candidate in candidates))


def resolve_model_path(config_path: pathlib.Path, value: str,
                       search_dirs: tuple[pathlib.Path, ...] = ()) -> pathlib.Path:
    """Resolve a scene path against several candidate directories.

    Order: the extra ``search_dirs``, the config file's directory, then the
    current working directory.
    """
    model_path = pathlib.Path(str(value)).expanduser()
    if model_path.is_absolute():
        return model_path
    for base in (*search_dirs, config_path.parent, pathlib.Path.cwd()):
        candidate = (base / model_path).resolve()
        if candidate.is_file():
            return candidate
    return (config_path.parent / model_path).resolve()


def load_config(path: pathlib.Path | None = None,
                model_path: str | None = None) -> SimConfig:
    """Read, coerce and validate the simulator configuration.

    ``model_path`` overrides the scene from the YAML; it is resolved the same
    way (relative to the config file first, then the current directory).
    """
    config_path = (path or default_config_path()).expanduser().resolve()
    with config_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    section = document
    if not isinstance(section, dict):
        raise ValueError(f'Invalid configuration in {config_path}')

    configured = str(section.get('model_path') or '')
    if model_path:
        # 命令行覆盖：先按 cwd、再按已配置场景所在目录（都是 mjcf/）、
        # 最后才回落到配置文件目录，这样 `--model obstacles.xml` 也能用。
        search_dirs: tuple[pathlib.Path, ...] = (pathlib.Path.cwd(),)
        if configured:
            search_dirs += (resolve_model_path(config_path, configured).parent,)
        scene = resolve_model_path(config_path, model_path, search_dirs)
    else:
        if not configured:
            raise ValueError(f'Missing "model_path" in {config_path}')
        scene = resolve_model_path(config_path, configured)

    config = SimConfig(
        model_path=scene,
        sim_frequency=float(section.get('sim_frequency', 500.0)),
        state_publish_frequency=float(section.get('state_publish_frequency', 100.0)),
        depth_publish_frequency=float(section.get('depth_publish_frequency', 20.0)),
        depth_width=int(section.get('depth_width', 106)),
        depth_height=int(section.get('depth_height', 60)),
        command_timeout=float(section.get('command_timeout', 0.25)),
        command_topic=str(section.get('command_topic', '/a1/command')),
        state_topic=str(section.get('state_topic', '/a1/state')),
        depth_topic=str(section.get('depth_topic', '/a1/depth')),
        reset_service=str(section.get('reset_service', '/a1/reset')),
        pause_service=str(section.get('pause_service', '/a1/pause')),
        resume_service=str(section.get('resume_service', '/a1/resume')),
        imu_topic=str(section.get('imu_topic', '/a1/imu')),
        imu_publish_frequency=float(section.get('imu_publish_frequency', 200.0)),
    )
    _validate(config, config_path)
    return config


def _validate(config: SimConfig, config_path: pathlib.Path) -> None:
    """Reject configurations that would make the loop behave unpredictably."""
    if config.sim_frequency <= 0.0:
        raise ValueError('sim_frequency must be greater than zero')
    if config.state_publish_frequency <= 0.0:
        raise ValueError('state_publish_frequency must be greater than zero')
    if config.depth_publish_frequency <= 0.0:
        raise ValueError('depth_publish_frequency must be greater than zero')
    if config.imu_publish_frequency <= 0.0:
        raise ValueError('imu_publish_frequency must be greater than zero')
    if config.depth_width <= 0 or config.depth_height <= 0:
        raise ValueError('depth_width and depth_height must be greater than zero')
    if not config.model_path.is_file():
        raise FileNotFoundError(f'Model file not found: {config.model_path} '
                                f'(referenced by {config_path})')
