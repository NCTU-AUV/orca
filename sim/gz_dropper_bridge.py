#!/usr/bin/env python3
"""放球機構的模擬：提供跟實機 ball_dropper（src/orca_actuators）一樣的 ROS 介面，放球時叫 Gazebo 放開球。

    ROS  /ball_dropper/set_hold   std_srvs/SetBool   false = 放開 → gz /orca/ball_dropper/release
    ROS  /ball_dropper/holding    std_msgs/Bool      是不是還吸著（transient local，每秒一次）

球掛在船上、放開的機制在世界檔（sim/worlds/sauvc_final.sdf 的 DetachableJoint）與
sim/models/dropper_ball；放開之後球有沒有掉進 drum 由 sim/referee.py 判斷。

跟實機不同的地方：放開是一次性的。DetachableJoint 沒有設 attach topic，球放掉就吸不回去，
set_hold true 會回 success=false。所以模擬重開（simulator 重建）時這個服務也要重開，
狀態才會回到「吸著」：deploy/sim/mission.sh、referee_check.sh 都會一起重開。
沒有球的世界（sauvc_qual.sdf 等）一樣回報「吸著」，收到放開指令不會有事發生。

在 orca-sim image 裡跑，見 deploy/sim/compose.yaml 的 sim_dropper 服務。
"""
import rclpy
from gz.msgs10.empty_pb2 import Empty as GzEmpty
from gz.transport13 import Node as GzNode
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

GZ_TOPIC = '/orca/ball_dropper/release'     # 要跟世界檔 DetachableJoint 的 <detach_topic> 一樣
HOLDING_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)   # 跟 orca_actuators 一樣


class DropperBridge(Node):
    def __init__(self):
        super().__init__('sim_dropper')
        self.gz = GzNode()
        self.gz_pub = self.gz.advertise(GZ_TOPIC, GzEmpty)
        self.holding = True
        self.pub = self.create_publisher(Bool, '/ball_dropper/holding', HOLDING_QOS)
        self.create_service(SetBool, '/ball_dropper/set_hold', self.on_set_hold)
        self.create_timer(1.0, self.publish)
        self.get_logger().info(f'/ball_dropper/set_hold（ROS）→ {GZ_TOPIC}（gz）')
        self.publish()

    def publish(self):
        self.pub.publish(Bool(data=self.holding))

    def on_set_hold(self, req, res):
        if req.data:
            res.success = self.holding
            res.message = '本來就是吸住' if self.holding else '模擬裡球放掉就吸不回去（重開模擬）'
            return res
        if self.holding:
            self.gz_pub.publish(GzEmpty())
            self.holding = False
            self.publish()
            self.get_logger().info('放球')
            res.message = '放開'
        else:
            res.message = '本來就是放開'
        res.success = True
        return res


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
