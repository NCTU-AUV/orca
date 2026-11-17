#!/usr/bin/env python3
"""放球指令：ROS → Gazebo。

    ROS  /ball_dropper/release   std_msgs/Empty   ← 任務層發這個
     ↓
    gz   /orca/ball_dropper/release   gz.msgs.Empty   → 世界檔裡的 DetachableJoint 放開球

**ROS 這一側的介面是暫定的**（2026-11-17）：實機的電磁鐵放球機構還沒定案，名稱與型別先用
/ball_dropper/release、std_msgs/Empty。實機介面定了之後，只要改下面 ROS_TOPIC、ROS_TYPE 這兩行
（型別不是 Empty 的話，on_release 裡決定什麼值算「放球」），任務層和這裡就對得上。

球掛在船上、放開的機制在世界檔（sim/worlds/sauvc_final.sdf 的 DetachableJoint）與
sim/models/dropper_ball；放開之後球有沒有掉進 drum 由 sim/referee.py 判斷。
放開是一次性的：DetachableJoint 沒有設 attach topic，球放掉就不會再吸回去，要重來就重開模擬。

在 orca-sim image 裡跑，見 deploy/sim/compose.yaml 的 sim_dropper 服務。
"""
import rclpy
from gz.msgs10.empty_pb2 import Empty as GzEmpty
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from std_msgs.msg import Empty

ROS_TOPIC = '/ball_dropper/release'         # 暫定，見檔頭
ROS_TYPE = Empty
GZ_TOPIC = '/orca/ball_dropper/release'     # 要跟世界檔 DetachableJoint 的 <detach_topic> 一樣


class DropperBridge(Node):
    def __init__(self):
        super().__init__('sim_dropper')
        self.gz = GzNode()
        self.pub = self.gz.advertise(GZ_TOPIC, GzEmpty)
        self.create_subscription(ROS_TYPE, ROS_TOPIC, self.on_release, 10)
        self.get_logger().info(f'{ROS_TOPIC}（ROS）→ {GZ_TOPIC}（gz）')

    def on_release(self, _msg):
        self.pub.publish(GzEmpty())
        self.get_logger().info('放球')


def main():
    rclpy.init()
    node = DropperBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
