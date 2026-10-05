#!/usr/bin/env python3
"""Live monitor for the A1 sim <-> deploy loop.

Logs robot state, IMU and the joint commands received on /a1/command, and
flags the failure modes that make a policy "suddenly lose balance":

  * command gaps  (the sim zeroes the torque watchdog after command_timeout)
  * large tilt    (|roll| or |pitch| beyond a threshold)
  * non-finite values
  * joint tracking error (commanded vs measured position)

Writes one CSV row per /a1/state message (100 Hz) to /tmp/a1_monitor_*.csv and
prints a live summary + immediate anomaly lines on stdout.
"""
from __future__ import annotations

import argparse
import math
import time
from datetime import datetime

import rclpy
from rclpy.node import Node

from a1_sim_msgs.msg import ImuData, RobotCmd, RobotData

JOINTS = ['FR_hip', 'FR_thigh', 'FR_calf',
          'FL_hip', 'FL_thigh', 'FL_calf',
          'RR_hip', 'RR_thigh', 'RR_calf',
          'RL_hip', 'RL_thigh', 'RL_calf']


class Monitor(Node):
    def __init__(self, path: str, tilt_limit: float, gap_warn: float, gap_crit: float) -> None:
        super().__init__('a1_monitor')
        self._file = open(path, 'w', buffering=1)
        self._tilt_limit = tilt_limit
        self._gap_warn = gap_warn
        self._gap_crit = gap_crit

        self._joints = len(JOINTS)
        self._cmd = None
        self._cmd_time = 0.0
        self._last_cmd_time = 0.0
        self._cmd_gap = float('inf')
        self._cmd_count = 0
        self._cmd_rate = 0.0
        self._rate_t0 = time.perf_counter()
        self._rate_n = 0
        self._imu = None
        self._imu_time = 0.0

        self._state_n = 0
        self._last_state_t = 0.0
        self._warn_n = 0
        self._crit_n = 0
        self._flags: set[str] = set()

        header = (['t', 'dt_state_ms', 'cmd_gap_ms', 'cmd_hz',
                   'roll', 'pitch', 'yaw', 'wx', 'wy', 'wz']
                  + [f'cmd_{j}' for j in JOINTS]
                  + [f'pos_{j}' for j in JOINTS]
                  + [f'vel_{j}' for j in JOINTS]
                  + [f'tau_{j}' for j in JOINTS]
                  + [f'err_{j}' for j in JOINTS])
        self._file.write(','.join(header) + '\n')

        self.create_subscription(RobotData, '/a1/state', self._on_state, 50)
        self.create_subscription(RobotCmd, '/a1/command', self._on_command, 50)
        self.create_subscription(ImuData, '/a1/imu', self._on_imu, 50)

        self._summary = self.create_timer(1.0, self._print_summary)
        self.get_logger().info(f'MONITOR READY -> {path}')

    # -- subscriptions ------------------------------------------------------
    def _on_command(self, msg: RobotCmd) -> None:
        now = time.perf_counter()
        if self._last_cmd_time:
            gap = now - self._last_cmd_time
            self._cmd_gap = gap
            if gap >= self._gap_crit:
                self._crit_n += 1
                print(f'!! COMMAND GAP {gap * 1000:.0f} ms '
                      f'(> {self._gap_crit * 1000:.0f} ms -> torque watchdog may zero control)')
            elif gap >= self._gap_warn:
                self._warn_n += 1
                print(f'!  command gap {gap * 1000:.0f} ms')
        self._last_cmd_time = now
        self._cmd = [(float(j.position), float(j.kp), float(j.kd), float(j.torque))
                     for j in msg.joints_cmd]
        self._cmd_time = now
        self._cmd_count += 1
        self._rate_n += 1

    def _on_imu(self, msg: ImuData) -> None:
        self._imu = (msg.orientation_w, msg.orientation_x, msg.orientation_y, msg.orientation_z,
                     msg.angular_velocity_x, msg.angular_velocity_y, msg.angular_velocity_z,
                     msg.roll, msg.pitch, msg.yaw)
        self._imu_time = time.perf_counter()
        self._check_tilt(msg.roll, msg.pitch)

    def _check_tilt(self, roll: float, pitch: float) -> None:
        if not (math.isfinite(roll) and math.isfinite(pitch)):
            self._flag('NON-FINITE imu')
            return
        if abs(pitch) > self._tilt_limit or abs(roll) > self._tilt_limit:
            key = 'TILT'
            if key not in self._flags:
                print(f'!! LARGE TILT roll={math.degrees(roll):.1f}deg '
                      f'pitch={math.degrees(pitch):.1f}deg')
            self._flag(key)

    def _flag(self, key: str) -> None:
        self._flags.add(key)

    # -- state sampling -----------------------------------------------------
    def _on_state(self, msg: RobotData) -> None:
        now = time.perf_counter()
        joints = msg.joints_data
        if len(joints) != self._joints:
            return
        dt_ms = (now - self._last_state_t) * 1000.0 if self._last_state_t else 0.0
        self._last_state_t = now

        cmd = self._cmd or [(0.0, 0.0, 0.0, 0.0)] * self._joints
        gap_ms = (now - self._last_cmd_time) * 1000.0 if self._last_cmd_time else float('inf')
        imu = self._imu or (1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

        row = [now, dt_ms, gap_ms, self._cmd_rate,
               imu[7], imu[8], imu[9], imu[4], imu[5], imu[6]]
        row += [cmd[i][0] for i in range(self._joints)]
        row += [float(j.position) for j in joints]
        row += [float(j.velocity) for j in joints]
        row += [float(j.torque) for j in joints]
        row += [cmd[i][0] - float(joints[i].position) for i in range(self._joints)]
        self._file.write(','.join(f'{v:.6g}' for v in row) + '\n')
        self._state_n += 1

    def _print_summary(self) -> None:
        self._cmd_rate = 0.0
        now = time.perf_counter()
        if self._rate_t0:
            self._cmd_rate = self._rate_n / (now - self._rate_t0 + 1e-9)
        self._rate_t0 = now
        self._rate_n = 0
        imu = self._imu
        tilt = ''
        if imu and math.isfinite(imu[7]) and math.isfinite(imu[8]):
            tilt = f'roll={math.degrees(imu[7]):6.1f} pitch={math.degrees(imu[8]):6.1f}'
        gap = (now - self._last_cmd_time) * 1000.0 if self._last_cmd_time else float('nan')
        print(f'[mon] states={self._state_n:7d} cmd={self._cmd_count:7d} '
              f'cmd_hz={self._cmd_rate:6.1f} gap={gap:6.1f}ms {tilt} '
              f'warn={self._warn_n} crit={self._crit_n}')

    def close(self) -> None:
        try:
            self._file.flush()
            self._file.close()
        except Exception:
            pass


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', default=None, help='CSV path (default /tmp/a1_monitor_<ts>.csv)')
    parser.add_argument('--tilt-deg', type=float, default=45.0)
    parser.add_argument('--gap-warn-ms', type=float, default=50.0)
    parser.add_argument('--gap-crit-ms', type=float, default=200.0)
    args = parser.parse_args()

    path = args.out or f"/tmp/a1_monitor_{datetime.now():%Y%m%d_%H%M%S}.csv"
    rclpy.init()
    node = Monitor(path, math.radians(args.tilt_deg),
                   args.gap_warn_ms / 1000.0, args.gap_crit_ms / 1000.0)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
