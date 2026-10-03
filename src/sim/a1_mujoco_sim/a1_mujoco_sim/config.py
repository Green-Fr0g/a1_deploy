"""Configuration loading and path resolution for the A1 MuJoCo simulator."""
from __future__ import annotations

import dataclasses
import pathlib

import yaml

PACKAGE_NAME = 'a1_mujoco_sim'


@dataclasses.dataclass(frozen=True)
class SimConfig:
    """Validated simulator configuration loaded from ``sim.yaml``."""

    model_path: pathlib.Path
    sim_frequency: float = 500.0
    state_publish_frequency: float = 100.0
    depth_publish_frequency: float = 20.0
    depth_width: int = 106
    depth_height: int = 60
    publish_depth: bool = True
    depth_camera: str = 'depth_camera'
    reset_keyframe: str = 'home'
    command_timeout: float = 0.25
    command_topic: str = '/a1/command'
    state_topic: str = '/a1/state'
    depth_topic: str = '/a1/depth'
    reset_service: str = '/a1/reset'

    @property
    def timestep(self) -> float:
        """Physics timestep derived from the configured simulation frequency."""
        return 1.0 / self.sim_frequency


def default_config_path() -> pathlib.Path:
    """Locate ``sim.yaml`` in a source checkout or an installed workspace."""
    package_dir = pathlib.Path(__file__).resolve().parent
    candidates = (
        package_dir.parent / 'config' / 'sim.yaml',
        package_dir.parents[1] / 'share' / PACKAGE_NAME / 'config' / 'sim.yaml',
        package_dir.parents[2] / 'share' / PACKAGE_NAME / 'config' / 'sim.yaml',
    )
    for candidate in candidates:
        if candidate.is_file():
            return candidate
    searched = '\n  '.join(str(candidate) for candidate in candidates)
    raise FileNotFoundError(f'Cannot find sim.yaml. Searched:\n  {searched}')


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
        publish_depth=bool(section.get('publish_depth', True)),
        depth_camera=str(section.get('depth_camera', 'depth_camera')),
        reset_keyframe=str(section.get('reset_keyframe', 'home')),
        command_timeout=float(section.get('command_timeout', 0.25)),
        command_topic=str(section.get('command_topic', '/a1/command')),
        state_topic=str(section.get('state_topic', '/a1/state')),
        depth_topic=str(section.get('depth_topic', '/a1/depth')),
        reset_service=str(section.get('reset_service', '/a1/reset')),
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
    if config.depth_width <= 0 or config.depth_height <= 0:
        raise ValueError('depth_width and depth_height must be greater than zero')
    if not config.model_path.is_file():
        raise FileNotFoundError(f'Model file not found: {config.model_path} '
                                f'(referenced by {config_path})')
