"""手柄按键链路测试：按短按时序注入 /joy，验证状态机是否都能切换。

用法（另一个终端里跑 `frog_deploy_main --robot a1 --mode sim`）：
    python3 tools/joy_tap_test.py
"""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy

TAP = 0.05  # 一次“短按”持续 50 ms，模拟真人快速点按


def main() -> int:
    rclpy.init()
    node = Node('joy_tap_test')
    publisher = node.create_publisher(Joy, '/joy', 10)

    def send(buttons, axes, note):
        message = Joy()
        message.buttons = buttons
        message.axes = axes
        # 连发两帧，避免单帧被 discovery / 队列丢掉
        for _ in range(2):
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.01)
        print(f't={time.time():.3f}  {note}')

    idle_buttons = [0] * 11
    idle_axes = [0.0] * 8

    time.sleep(1.0)  # 等发现完成

    # 1) 短按 A → GetUp
    send([1] + [0] * 10, idle_axes, 'tap A (GetUp)')
    time.sleep(TAP)
    send(idle_buttons, idle_axes, 'release')

    # 2) 等站起插值完成（1.5 s）后短按方向键下 → WalkTheseWay
    time.sleep(2.0)
    send(idle_buttons, [0.0] * 7 + [-1.0], 'tap DPadDown (WalkTheseWay)')
    time.sleep(TAP)
    send(idle_buttons, idle_axes, 'release')

    # 3) 短按 X → Passive
    time.sleep(1.0)
    send([0, 0, 1] + [0] * 8, idle_axes, 'tap X (Passive)')
    time.sleep(TAP)
    send(idle_buttons, idle_axes, 'release')

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
