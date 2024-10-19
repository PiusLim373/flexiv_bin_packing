#!/usr/bin/python3
import sys
from ament_index_python import get_package_share_directory
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from std_srvs.srv import Trigger, SetBool

package_share_directory = get_package_share_directory("motion_server")
sys.path.insert(0, package_share_directory)
import flexivrdk

import rclpy
from rclpy.node import Node
import time
from scipy.spatial.transform import Rotation as R
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import JointState
MOCK_ROBOT = True


class MotionServer(Node):

    def __init__(self):
        super().__init__("motion_server")
        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(0.01, self.timer_callback)


    # timer callback
    def timer_callback(self):
        robot_joints = ['-0.07751', '-0.23965', '0.10633', '1.6650', '-0.02561', '0.14115', '0.03355']
        data = JointState()
        data.header.stamp = self.get_clock().now().to_msg()
        data.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"]
        for x in robot_joints:
            data.position.append(float(x))
        # data.position = [-0.07751, -0.23965, 0.10633, 1.47650, -0.02561, 0.14115, 0.03355]
        self.joint_state_pub.publish(data)

 

def main(args=None):
    rclpy.init(args=args)
    motion_server = MotionServer()
    rclpy.spin(motion_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# ros2 service call /move_l bin_packing_msgs/srv/MoveL '{"mode": "euler", "euler_pose": {"position": {"x": 0.7, "y": -0.1, "z": 0.3}, "orientation": {"x": 180, "y": 0, "z": 180}}}'
