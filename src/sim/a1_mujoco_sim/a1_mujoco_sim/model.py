"""MuJoCo model wrapper for the Unitree A1."""
from __future__ import annotations

import dataclasses
import pathlib
from collections.abc import Iterable, Sequence

import mujoco
import numpy as np

# Sensors carrying the trunk-mounted IMU, in the order used by the XML model.
IMU_SENSORS = ('imu_quat', 'imu_gyro', 'imu_acc')


@dataclasses.dataclass(frozen=True)
class ImuSample:
    """One IMU reading taken from the trunk-mounted sensor site."""

    orientation: np.ndarray          # w, x, y, z; IMU orientation in world
    angular_velocity: np.ndarray     # rad/s, in the IMU frame
    linear_acceleration: np.ndarray  # m/s^2, in the IMU frame, includes gravity


class RobotModel:
    """Owns the MuJoCo model/data pair and the actuator-to-joint mapping.

    Every raw index lookup lives here so that the ROS node and the simulation
    loop only deal with joint level positions, velocities and torques.
    """

    def __init__(self, model_path: pathlib.Path, timestep: float) -> None:
        if timestep <= 0.0:
            raise ValueError('timestep must be greater than zero')
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.model.opt.timestep = timestep
        self.data = mujoco.MjData(self.model)

        # Actuators are the authoritative joint order used by the ROS topics.
        self.joint_ids = np.asarray(self.model.actuator_trnid[:, 0], dtype=np.int32)
        self.dof_ids = np.asarray(
            [self.model.jnt_dofadr[joint_id] for joint_id in self.joint_ids],
            dtype=np.int32,
        )

        # Sensor addresses are resolved once so the publish callbacks stay cheap.
        self._sensor_addresses: dict[str, tuple[int, int]] = {}
        for name in IMU_SENSORS:
            address = self.sensor_address(name)
            if address is not None:
                self._sensor_addresses[name] = address

    @property
    def num_actuators(self) -> int:
        return int(self.model.nu)

    @property
    def actuator_names(self) -> list[str]:
        names = (mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, index)
                 for index in range(self.num_actuators))
        return [name for name in names]

    def camera_id(self, name: str) -> int:
        """Return a camera id, or ``-1`` when the model has no such camera."""
        return int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, name))

    def camera_fovy(self, name: str) -> float:
        """Return the vertical field of view of a camera in degrees."""
        camera_id = self.camera_id(name)
        return float(self.model.cam_fovy[camera_id]) if camera_id >= 0 else 0.0

    def zero_control(self) -> None:
        self.data.ctrl[:] = 0.0

    def apply_joint_commands(self, commands: Sequence[object]) -> None:
        """Write feed-forward torque plus optional PD terms to the actuators.

        ``commands`` items must expose ``torque``, ``kp``, ``position``, ``kd``
        and ``velocity``, matching ``a1_sim_msgs/msg/JointCmd``.
        """
        self.zero_control()
        for index in range(min(len(commands), self.num_actuators)):
            command = commands[index]
            joint_id = int(self.joint_ids[index])
            dof_id = int(self.dof_ids[index])

            torque = float(command.torque)
            if command.kp:
                position = float(self.data.qpos[self.model.jnt_qposadr[joint_id]])
                torque += float(command.kp) * (float(command.position) - position)
            if command.kd:
                velocity = float(self.data.qvel[dof_id])
                torque += float(command.kd) * (float(command.velocity) - velocity)

            self.data.ctrl[index] = torque

    def read_joint_states(self) -> Iterable[tuple[float, float, float]]:
        """Yield ``(position, velocity, torque)`` for every actuator joint."""
        for index, joint_id in enumerate(self.joint_ids):
            dof_id = int(self.dof_ids[index])
            yield (
                float(self.data.qpos[self.model.jnt_qposadr[joint_id]]),
                float(self.data.qvel[dof_id]),
                float(self.data.qfrc_actuator[dof_id]),
            )

    def reset_to_keyframe(self, keyframe: str) -> None:
        """Reset the simulation state, falling back to the model default."""
        key_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, keyframe)
        if key_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, key_id)
        else:
            mujoco.mj_resetData(self.model, self.data)
        self.zero_control()

    def step(self) -> None:
        mujoco.mj_step(self.model, self.data)

    def body_pose(self, name: str) -> tuple[np.ndarray, np.ndarray] | None:
        """Return ``(position, rotation)`` of a body, or ``None`` if absent."""
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, name)
        if body_id < 0:
            return None
        position = np.array(self.data.xpos[body_id], dtype=float, copy=True)
        rotation = np.array(self.data.xmat[body_id], dtype=float, copy=True).reshape(3, 3)
        return position, rotation

    # -- IMU sensors --------------------------------------------------------
    @property
    def imu_available(self) -> bool:
        """Whether the model provides the full set of IMU sensors."""
        return all(name in self._sensor_addresses for name in IMU_SENSORS)

    def sensor_address(self, name: str) -> tuple[int, int] | None:
        """Return the ``(sensordata offset, dimension)`` of a named sensor."""
        sensor_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
        if sensor_id < 0:
            return None
        return int(self.model.sensor_adr[sensor_id]), int(self.model.sensor_dim[sensor_id])

    def read_imu(self) -> ImuSample | None:
        """Return the latest IMU sample, or ``None`` when the model lacks one."""
        if not self.imu_available:
            return None
        return ImuSample(
            orientation=self._sensor_values('imu_quat'),
            angular_velocity=self._sensor_values('imu_gyro'),
            linear_acceleration=self._sensor_values('imu_acc'),
        )

    def _sensor_values(self, name: str) -> np.ndarray:
        start, dimension = self._sensor_addresses[name]
        return np.array(self.data.sensordata[start:start + dimension], dtype=float, copy=True)
