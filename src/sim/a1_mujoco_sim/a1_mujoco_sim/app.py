"""Application wiring: viewer, simulation loop, signals and shutdown."""
from __future__ import annotations

import contextlib
import os
import signal
import threading
import time

import rclpy

from .config import DEPTH_CAMERA_NAME, SimConfig
from .depth import DepthWindow, DepthWindowError
from .model import RobotModel
from .node import A1MujocoSimNode
from .viewer import FollowCamera


class ViewerError(RuntimeError):
    """Raised when the MuJoCo viewer cannot be opened."""


class SimulatorApp:
    """Runs the physics loop next to a ROS 2 executor thread.

    Closing the MuJoCo window, pressing Ctrl+C and receiving SIGTERM all lead
    to the same shutdown path: the loop stops, ROS is shut down, the executor
    thread is joined and every graphical resource is released.
    """

    def __init__(self, config: SimConfig, *, show_viewer: bool = True,
                 follow_camera: bool = False, show_depth: bool = False,
                 depth_scale: int = 1) -> None:
        self._config = config
        self._show_viewer = show_viewer
        self._follow_camera = follow_camera
        self._show_depth = show_depth
        self._depth_scale = depth_scale
        self._stop = threading.Event()
        self._depth_window: DepthWindow | None = None
        self._next_depth_time = 0.0

    def run(self) -> int:
        """Start the simulator and block until it is asked to stop."""
        self._stop.clear()
        self._next_depth_time = 0.0
        rclpy.init()
        node = None
        executor = None
        try:
            robot = RobotModel(self._config.model_path, self._config.timestep)
            node = A1MujocoSimNode(self._config, robot)
            node.initialize_depth_renderer()
            self._log_startup(node, robot)
            if self._show_depth:
                self._depth_window = DepthWindow(
                    self._config.depth_width, self._config.depth_height,
                    scale=self._depth_scale)

            executor = threading.Thread(target=_spin_ros, args=(node,),
                                        name='ros-executor', daemon=True)
            executor.start()
            with _signal_handlers(self._stop):
                viewer = self._open_viewer(robot)
                try:
                    self._loop(robot, node, viewer)
                finally:
                    _close_viewer(viewer)
        finally:
            self._shutdown(node, executor)
        return 0

    # -- main loop ----------------------------------------------------------
    def _loop(self, robot: RobotModel, node: A1MujocoSimNode, viewer) -> None:
        config = self._config
        camera = FollowCamera() if self._follow_camera else None
        depth_period = 1.0 / config.depth_publish_frequency
        next_step = time.perf_counter()

        while not self._stop.is_set() and rclpy.ok():
            if viewer is not None and not viewer.is_running():
                break  # the user closed the viewer window
            with node.lock:
                if node.command_is_stale():
                    robot.zero_control()
                robot.step()
                sim_time = robot.data.time
            if viewer is not None:
                if camera is not None:
                    camera.update(viewer, robot)
                try:
                    viewer.sync()
                except Exception:
                    break  # the viewer was closed while syncing
            if sim_time >= self._next_depth_time:
                self._next_depth_time = sim_time + depth_period
                self._publish_depth(node)

            next_step = self._advance_clock(next_step, config.timestep)

    @staticmethod
    def _advance_clock(next_step: float, timestep: float) -> float:
        """Sleep until the next simulation instant, avoiding clock drift."""
        next_step += timestep
        remaining = next_step - time.perf_counter()
        if remaining > 0.0:
            time.sleep(remaining)
            return next_step
        return time.perf_counter()

    def _publish_depth(self, node: A1MujocoSimNode) -> None:
        """Render and publish depth, optionally mirroring it into the window."""
        depth = node.render_and_publish_depth()
        if depth is None or self._depth_window is None:
            return
        try:
            self._depth_window.show(depth)
        except DepthWindowError as error:
            node.get_logger().warning(f'Depth window disabled: {error}')
            self._depth_window.close()
            self._depth_window = None

    # -- viewer -------------------------------------------------------------
    def _open_viewer(self, robot: RobotModel):
        if not self._show_viewer:
            return None
        # MuJoCo terminates the process when GLFW cannot initialise, which
        # cannot be caught, so reject a missing display before it is called.
        if not _has_display():
            raise ViewerError(
                'No display detected (DISPLAY and WAYLAND_DISPLAY are unset). '
                'Use --headless on machines without a graphical session.')
        try:
            import mujoco.viewer

            return mujoco.viewer.launch_passive(robot.model, robot.data)
        except Exception as error:
            raise ViewerError(f'Cannot open the MuJoCo viewer: {error}') from error

    # -- startup and shutdown ----------------------------------------------
    def _log_startup(self, node: A1MujocoSimNode, robot: RobotModel) -> None:
        config = self._config
        node.get_logger().info(
            f'Loaded {config.model_path} ({robot.model.nq} qpos, '
            f'{robot.num_actuators} actuators, timestep={config.timestep:.6f}s)')
        fovy = robot.camera_fovy(DEPTH_CAMERA_NAME)
        if fovy > 0.0:
            node.get_logger().info(
                f'Depth camera {DEPTH_CAMERA_NAME!r}: vertical FOV {fovy:.4f} deg')
        if not self._show_viewer:
            node.get_logger().info('Viewer: disabled (--headless)')
        elif self._follow_camera:
            node.get_logger().info('Viewer: open, camera follows the robot')
        else:
            node.get_logger().info('Viewer: open, free camera (--camera to follow the robot)')
        if self._show_depth:
            node.get_logger().info(
                'Depth window: open, bottom right corner and kept on top')
        else:
            node.get_logger().info(
                'Depth window: closed (pass --depth to show the depth image)')
        if robot.imu_available:
            node.get_logger().info(
                f'IMU: publishing {config.imu_topic!r} at '
                f'{config.imu_publish_frequency:.0f} Hz')
        else:
            node.get_logger().warning(
                'IMU: model has no imu_quat/imu_gyro/imu_acc sensors')
            node.get_logger().warning(
                f'IMU: {config.imu_topic} will not be published')

    def _shutdown(self, node, executor: threading.Thread | None) -> None:
        self._stop.set()
        if self._depth_window is not None:
            self._depth_window.close()
            self._depth_window = None
        if node is not None:
            node.close_renderer()
        if rclpy.ok():
            rclpy.shutdown()
        if executor is not None:
            executor.join(timeout=2.0)
        if node is not None:
            node.destroy_node()


def _spin_ros(node: A1MujocoSimNode) -> None:
    """Run the ROS executor until the context is shut down."""
    try:
        rclpy.spin(node)
    except rclpy.executors.ExternalShutdownException:
        pass


@contextlib.contextmanager
def _signal_handlers(stop: threading.Event):
    """Turn Ctrl+C (SIGINT) and SIGTERM into a clean stop request."""

    def handler(signum, frame):
        del signum, frame
        stop.set()

    installed = {sig: signal.signal(sig, handler)
                 for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        yield
    finally:
        for sig, previous in installed.items():
            signal.signal(sig, previous)


def _close_viewer(viewer) -> None:
    if viewer is None:
        return
    try:
        viewer.close()
    except Exception:
        pass


def _has_display() -> bool:
    """Whether a graphical session is available for the viewer window."""
    return bool(os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY'))
