"""ROS 2 interface: state publishing, command subscription and reset service."""
from __future__ import annotations

import threading
from array import array

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node

from a1_sim_msgs.msg import DepthImage, JointData, RobotCmd, RobotData
from a1_sim_msgs.srv import Reset
from sensor_msgs.msg import Imu

from .config import DEPTH_CAMERA_NAME, IMU_FRAME_ID, RESET_KEYFRAME, SimConfig
from .model import RobotModel


class A1MujocoSimNode(Node):
    """Bridges the MuJoCo simulation to ROS 2 topics and services.

    ``lock`` guards ``robot.data``: the callbacks below and the simulation loop
    must both hold it while reading or writing physics state.
    """

    def __init__(self, config: SimConfig, robot: RobotModel) -> None:
        super().__init__('a1_mujoco_sim_node')
        self._config = config
        self._robot = robot
        self.lock = threading.Lock()
        self._last_command_time = self.get_clock().now()
        self._renderer = None
        self._renderer_unavailable = False

        self._state_publisher = self.create_publisher(RobotData, config.state_topic, 10)
        self._depth_publisher = self.create_publisher(DepthImage, config.depth_topic, 10)
        self.create_subscription(RobotCmd, config.command_topic, self._on_command, 10)
        self.create_service(Reset, config.reset_service, self._on_reset)
        self._state_timer = self.create_timer(
            1.0 / config.state_publish_frequency, self._publish_state)

        self._imu_publisher = None
        self._imu_timer = None
        if robot.imu_available:
            self._imu_publisher = self.create_publisher(Imu, config.imu_topic, 10)
            self._imu_timer = self.create_timer(
                1.0 / config.imu_publish_frequency, self._publish_imu)

    # -- command watchdog ---------------------------------------------------
    def command_is_stale(self) -> bool:
        """Whether no command arrived recently.

        Must be called while holding ``lock``. A ``command_timeout`` of zero or
        less disables the watchdog.
        """
        if self._config.command_timeout <= 0.0:
            return False
        elapsed = (self.get_clock().now() - self._last_command_time).nanoseconds * 1e-9
        return elapsed > self._config.command_timeout

    # -- depth sensor -------------------------------------------------------
    def render_and_publish_depth(self) -> np.ndarray | None:
        """Render the depth camera, publish it and return the raw image."""
        camera_id = self._robot.camera_id(DEPTH_CAMERA_NAME)
        if camera_id < 0:
            return None
        renderer = self._get_renderer()
        if renderer is None:
            return None

        with self.lock:
            renderer.enable_depth_rendering()
            renderer.update_scene(self._robot.data, camera=camera_id)
            depth = np.asarray(renderer.render(), dtype=np.float32)
            renderer.disable_depth_rendering()
            message = self._depth_message(depth)
        self._depth_publisher.publish(message)
        return depth

    def close_renderer(self) -> None:
        if self._renderer is not None:
            try:
                self._renderer.close()
            except Exception:
                pass
            self._renderer = None

    # -- callbacks ----------------------------------------------------------
    def _on_command(self, message: RobotCmd) -> None:
        with self.lock:
            self._robot.apply_joint_commands(message.joints_cmd)
            self._last_command_time = self.get_clock().now()

    def _publish_state(self) -> None:
        with self.lock:
            message = RobotData()
            message.stamp = self.get_clock().now().to_msg()
            for position, velocity, torque in self._robot.read_joint_states():
                joint = JointData()
                joint.position = position
                joint.velocity = velocity
                joint.torque = torque
                message.joints_data.append(joint)
        self._state_publisher.publish(message)

    def _on_reset(self, request, response):
        del request
        with self.lock:
            self._robot.reset_to_keyframe(RESET_KEYFRAME)
            self._last_command_time = self.get_clock().now()
        response.result = 0
        return response

    def _publish_imu(self) -> None:
        """Publish the trunk IMU as ``sensor_msgs/Imu``.

        Covariances stay at zero because the MuJoCo sensors are noiseless.
        """
        with self.lock:
            sample = self._robot.read_imu()
            if sample is None:
                return
            message = Imu()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = IMU_FRAME_ID
            message.orientation.w = float(sample.orientation[0])
            message.orientation.x = float(sample.orientation[1])
            message.orientation.y = float(sample.orientation[2])
            message.orientation.z = float(sample.orientation[3])
            gyroscope = sample.angular_velocity
            message.angular_velocity.x = float(gyroscope[0])
            message.angular_velocity.y = float(gyroscope[1])
            message.angular_velocity.z = float(gyroscope[2])
            accelerometer = sample.linear_acceleration
            message.linear_acceleration.x = float(accelerometer[0])
            message.linear_acceleration.y = float(accelerometer[1])
            message.linear_acceleration.z = float(accelerometer[2])
        self._imu_publisher.publish(message)

    # -- helpers ------------------------------------------------------------
    def _get_renderer(self):
        """Create the depth renderer on first use, disabling it on failure."""
        if self._renderer is not None or self._renderer_unavailable:
            return self._renderer
        try:
            self._renderer = mujoco.Renderer(
                self._robot.model,
                height=self._config.depth_height,
                width=self._config.depth_width,
            )
        except Exception as error:  # missing OpenGL/EGL backend
            self._renderer_unavailable = True
            self.get_logger().warning(f'Depth rendering disabled: {error}')
        return self._renderer

    def _depth_message(self, depth: np.ndarray) -> DepthImage:
        message = DepthImage()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = DEPTH_CAMERA_NAME
        message.height, message.width = (int(size) for size in depth.shape)
        message.encoding = '32FC1'
        message.is_bigendian = 0
        message.step = message.width * depth.dtype.itemsize
        message.data = array('B', depth.tobytes())
        return message
