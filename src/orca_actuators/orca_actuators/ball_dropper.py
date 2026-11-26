#!/usr/bin/env python3
"""ball_dropper：放球機構的電磁鐵（Jetson 40-pin 的一支 GPIO 輸出）。

    ros2 run orca_actuators ball_dropper --ros-args --params-file config/ball_dropper.yaml

介面（模擬的 sim/gz_dropper_bridge.py 也提供同一套，任務層不用分實機、模擬）：

    /ball_dropper/set_hold   std_srvs/SetBool   true = 吸住（通電），false = 放開（斷電）
                                                回覆在腳位設好之後才送，success=false 表示沒做到
    /ball_dropper/holding    std_msgs/Bool      目前是不是吸住。變化時馬上發，之後每秒一次；
                                                transient local：晚訂閱的也拿得到最後一筆

參數：pin（40-pin 實體腳號）、active_high（HIGH = 吸住）、hold_on_start（啟動就吸住）。
每次改變都插一筆標記（/session/marker），rosbag 裡查得到什麼時候放的球。

- 腳位由這個節點佔住：tools/hw_tests/magnet.py 要等這個節點停了才能用。
- 節點結束（docker stop、Ctrl-C、掛掉）時不改腳位，只放掉它。放掉之後腳位維持什麼電位
  沒有保證，要用 magnet.py 的 exit 實測；所以這個節點要 restart: unless-stopped，任務中別重開它。
- 這裡只知道「要求輸出的值」，沒有感測器確認球真的掉了。
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

from orca_session.session import Marker

from .gpio import GpioError, OutputPin

HOLDING_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)


class BallDropper(Node):
    def __init__(self):
        super().__init__('ball_dropper')
        pin = self.declare_parameter('pin', 29).value
        active_high = self.declare_parameter('active_high', True).value
        hold = self.declare_parameter('hold_on_start', True).value
        self.pin = OutputPin(pin, active_high=active_high, initial=hold, consumer='ball_dropper')
        self.mark = Marker(self, 'ball_dropper')
        self.pub = self.create_publisher(Bool, '/ball_dropper/holding', HOLDING_QOS)
        self.create_service(SetBool, '/ball_dropper/set_hold', self.on_set_hold)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(
            f'pin {pin}（{self.pin.name}），{"HIGH" if active_high else "LOW"} = 吸住；'
            f'啟動時{"吸住" if hold else "放開"}')
        self.publish()

    def publish(self):
        self.pub.publish(Bool(data=self.pin.value))

    def on_set_hold(self, req, res):
        want = bool(req.data)
        word = '吸住' if want else '放開'
        try:
            changed = self.pin.value != want
            self.pin.set(want)
        except (GpioError, OSError) as e:
            res.success, res.message = False, f'{word}失敗：{e}'
            self.get_logger().error(res.message)
            return res
        self.publish()
        if changed:
            self.mark(word)
        res.success, res.message = True, word if changed else f'本來就是{word}'
        return res

    def destroy_node(self):
        self.get_logger().info(f'結束：腳位不改（最後是{"吸住" if self.pin.value else "放開"}），放掉')
        self.pin.close()
        super().destroy_node()


def main():
    rclpy.init()
    try:
        node = BallDropper()
    except GpioError as e:
        print(f'ball_dropper 啟動失敗：{e}', flush=True)
        rclpy.try_shutdown()
        return 1
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    main()
