#!/usr/bin/python3
import sys
from ament_index_python import get_package_share_directory
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *

package_share_directory = get_package_share_directory("motion_server")
sys.path.insert(0, package_share_directory)
import flexivrdk

import rclpy
from rclpy.node import Node
import time
from scipy.spatial.transform import Rotation as R


class MotionServer(Node):

    def __init__(self):
        super().__init__("motion_server")
        self.log = flexivrdk.Log()
        self.mode = flexivrdk.Mode
        self.plan_info = flexivrdk.PlanInfo()
        self.robot = flexivrdk.Robot("192.168.86.121", "192.168.86.120")
        self.enable_robot()

        self.srv = self.create_service(MoveJ, "move_j", self.move_j_cb)
        self.srv = self.create_service(MoveL, "move_l", self.move_l_cb)
        self.srv = self.create_service(MoveRelative, "move_relative", self.move_relative_cb)

    # Robot internal functions
    def enable_robot(self):
        self.robot.setMode(self.mode.NRT_PRIMITIVE_EXECUTION)
        # Clear fault on robot server if any
        if self.robot.isFault():
            self.get_logger().warn("Fault occurred on self.robot server, trying to clear ...")
            # Try to clear the fault
            self.robot.clearFault()
            time.sleep(2)
            # Check again
            if self.robot.isFault():
                self.get_logger().error("Fault cannot be cleared, exiting ...")
                return
            self.get_logger().info("Fault on robot server is cleared")

        # Enable the robot, make sure the E-stop is released before enabling
        self.get_logger().info("Enabling robot ...")
        self.robot.enable()

        # Wait for the robot to become operational
        while not self.robot.isOperational():
            time.sleep(1)
            # might need a timeout here
        self.get_logger().info("Robot is now operational")

    def parse_pt_states(self, pt_states, parse_target):
        """
        Parse the value of a specified primitive state from the pt_states string list.

        Parameters
        ----------
        pt_states : str list
            Primitive states string list returned from Robot::getPrimitiveStates().
        parse_target : str
            Name of the primitive state to parse for.

        Returns
        ----------
        str
            Value of the specified primitive state in string format. Empty string is
            returned if parse_target does not exist.
        """
        for state in pt_states:
            # Split the state sentence into words
            words = state.split()

            if words[0] == parse_target:
                return words[-1]

        return ""

    def quat2eulerZYX(self, quat, degree=False):
        """
        Convert quaternion to Euler angles with ZYX axis rotations.

        Parameters
        ----------
        quat : float list
            Quaternion input in [w,x,y,z] order.
        degree : bool
            Return values in degrees, otherwise in radians.

        Returns
        ----------
        float list
            Euler angles in [x,y,z] order, radian by default unless specified otherwise.
        """

        # Convert target quaternion to Euler ZYX using scipy package's 'xyz' extrinsic rotation
        # NOTE: scipy uses [x,y,z,w] order to represent quaternion
        eulerZYX = R.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_euler("xyz", degrees=degree).tolist()

        return eulerZYX

    # service callbacks
    def move_j_cb(self, request, response):
        joint_list = " ".join(map(str, request.joints))
        self.get_logger().info(f"Incoming Move J request: {request.joints}")
        if len(request.joints) != 7:
            self.get_logger().error(f"Move J must consists of 7 value")
            response.success = False
            return response
        # robot.executePrimitive("MoveJ(target=30 -45 0 90 0 40 30)")
        self.robot.executePrimitive(f"MoveJ(target={joint_list})")

        while self.parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1":
            if self.robot.isFault():
                self.get_logger().error("Robot is at fault state")
                response.success = False
                return response
            time.sleep(1)
        response.success = True
        return response

    def move_l_cb(self, request, response):
        self.get_logger().info(f"Incoming Move L request: {request.pose}")

        # robot.executePrimitive("MoveL(target=0.65 -0.3 0.2 180 0 180 WORLD WORLD_ORIGIN, maxVel=0.2")
        if request.mode == MoveL().Request.QUATERNION:
            quaternion_list = [
                request.quaternion_pose.quaternion.w,
                request.quaternion_pose.quaternion.x,
                request.quaternion_pose.quaternion.y,
                request.quaternion_pose.quaternion.z,
            ]
            eulerZYX_deg = self.quat2eulerZYX(quaternion_list, degree=True)
            self.robot.executePrimitive(
                f"MoveL(target={request.quaternion_pose.position.x} {request.quaternion_pose.position.y} {request.quaternion_pose.position.z} {eulerZYX_deg[0]} {eulerZYX_deg[1]} {eulerZYX_deg[2]} WORLD WORLD_ORIGIN, maxVel=0.2)"
            )

        elif request.mode == MoveL().Request.EULER:
            self.robot.executePrimitive(
                f"MoveL(target={request.euler_pose.position.x} {request.euler_pose.position.y} {request.euler_pose.position.z} {request.euler_pose.orientation.x} {request.euler_pose.orientation.y} {request.euler_pose.orientation.z} WORLD WORLD_ORIGIN, maxVel=0.2)"
            )

        while self.parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1":
            if self.robot.isFault():
                self.get_logger().error("Robot is at fault state")
                response.success = False
                return response
            time.sleep(1)
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    motion_server = MotionServer()
    rclpy.spin(motion_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
