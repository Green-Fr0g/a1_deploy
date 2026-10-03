"""Configuration loading and path resolution for the A1 MuJoCo simulator."""
from __future__ import annotations

import dataclasses
import pathlib

import yaml

# These names are part of the A1 model/ROS contract, rather than runtime
# configuration. Keep them here so callers do not duplicate string literals.
DEPTH_CAMERA_NAME = 'depth_camera'
IMU_FRAME_ID = 'imu'
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
    #depth camera
    depth_width: int = 106
    depth_height: int = 60
    command_timeout: float = 0.25

    @property
    def timestep(self) -> float:
        """Physics timestep derived from the configured simulation frequency."""
        return 1.0 / self.sim_frequency


def default_config_path() -> pathlib.Path:
    """Locate the YAML for both a source checkout and an installed package."""
    package_dir = pathlib.Path(__file__).resolve().parent
    candidates = (
        package_dir.parent / 'config' / 'sim.yaml',
        package_dir.parents[2] / 'share' / 'a1_mujoco_sim' / 'config' / 'sim.yaml',
        package_dir.parents[3] / 'share' / 'a1_mujoco_sim' / 'config' / 'sim.yaml',
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = '\n  '.join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f'Cannot find simulator configuration. Searched:\n  {searched}')


def resolve_model_path(config_path: pathlib.Path, value: str) -> pathlib.Path:
    """Resolve ``model_path`` relative to the config file, then to the cwd."""
    model_path = pathlib.Path(str(value)).expanduser()
    if model_path.is_absolute():
        return model_path
    for base in (config_path.parent, pathlib.Path.cwd()):
        candidate = (base / model_path).resolve()
        if candidate.is_file():
            return candidate
    return (config_path.parent / model_path).resolve()


def load_config(path: pathlib.Path | None = None) -> SimConfig:
    """Read, coerce and validate the simulator configuration."""
    config_path = (path or default_config_path()).expanduser().resolve()
    with config_path.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream) or {}
    section = document.get('a1_mujoco_sim_node', document.get('a1_mujoco_sim', document))
    if not isinstance(section, dict):
        raise ValueError(f'Invalid configuration in {config_path}')

    if 'model_path' not in section:
        raise ValueError(f'Missing "model_path" in {config_path}')

    config = SimConfig(
        model_path=resolve_model_path(config_path, section['model_path']),
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
