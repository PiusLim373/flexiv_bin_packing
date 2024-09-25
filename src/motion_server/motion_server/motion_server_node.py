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

MOCK_ROBOT = False


class MotionServer(Node):

    def __init__(self):
        super().__init__("motion_server")
        if not MOCK_ROBOT:
            self.log = flexivrdk.Log()
            self.mode = flexivrdk.Mode
            self.plan_info = flexivrdk.PlanInfo()
            self.robot_states = flexivrdk.RobotStates()
            self.robot = flexivrdk.Robot("192.168.3.100", "192.168.3.101")  # robot ip, local ip
            self.enable_robot()

        self.br = tf2_ros.TransformBroadcaster(self)
        self.srv = self.create_service(MoveJ, "move_j", self.move_j_cb)
        self.srv = self.create_service(MoveL, "move_l", self.move_l_cb)
        self.srv = self.create_service(Trigger, "home", self.home_cb)
        self.srv = self.create_service(Trigger, "fm_home", self.fm_home_cb)
        self.srv = self.create_service(MoveRelative, "move_relative", self.move_relative_cb)
        self.srv = self.create_service(Trigger, "contact", self.contact_cb)
        self.srv = self.create_service(SetBool, "gripper_control", self.gripper_control_cb)
        if not MOCK_ROBOT:
            self.robot.executePrimitive("ZeroFTSensor()")
            time.sleep(0.5)
            self.robot.executePrimitive("MoveJ(target=-1.05 -13.19 2.11 85.09 -0.52 8.1 1.3)")
            self.create_timer(0.01, self.timer_callback)

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

    # timer callback
    def timer_callback(self):
        self.robot.getRobotStates(self.robot_states)
        tcp_pose = self.robot_states.tcpPose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "tcp"
        t.transform.translation.x = tcp_pose[0]
        t.transform.translation.y = tcp_pose[1]
        t.transform.translation.z = tcp_pose[2]
        t.transform.rotation.w = tcp_pose[3]
        t.transform.rotation.x = tcp_pose[4]
        t.transform.rotation.y = tcp_pose[5]
        t.transform.rotation.z = tcp_pose[6]
        self.br.sendTransform(t)

        flange_pose = self.robot_states.flangePose
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "flange"
        t.transform.translation.x = flange_pose[0]
        t.transform.translation.y = flange_pose[1]
        t.transform.translation.z = flange_pose[2]
        t.transform.rotation.w = flange_pose[3]
        t.transform.rotation.x = flange_pose[4]
        t.transform.rotation.y = flange_pose[5]
        t.transform.rotation.z = flange_pose[6]
        self.br.sendTransform(t)

    # service callbacks
    def move_j_cb(self, request, response):
        joint_list = " ".join(map(str, request.joints))
        self.get_logger().info(f"Incoming Move J request: {request.joints}")
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response
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
        self.get_logger().info(f"Incoming Move L request: {request.euler_pose}")
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response
        # robot.executePrimitive("MoveL(target=0.65 -0.3 0.2 180 0 180 WORLD WORLD_ORIGIN, maxVel=0.2")
        if request.mode == MoveL.Request.QUATERNION:
            quaternion_list = [
                request.quaternion_pose.orientation.w,
                request.quaternion_pose.orientation.x,
                request.quaternion_pose.orientation.y,
                request.quaternion_pose.orientation.z,
            ]
            eulerZYX_deg = self.quat2eulerZYX(quaternion_list, degree=True)
            self.robot.executePrimitive(
                f"MoveL(target={request.quaternion_pose.position.x} {request.quaternion_pose.position.y} {request.quaternion_pose.position.z} {eulerZYX_deg[0]} {eulerZYX_deg[1]} {eulerZYX_deg[2]} WORLD WORLD_ORIGIN, maxVel=0.2)"
            )

        elif request.mode == MoveL.Request.EULER:
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

    def home_cb(self, request, response):
        self.get_logger().info(f"Incoming homing request")
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response

        # robot.executePrimitive("MoveJ(target=30 -45 0 90 0 40 30)")
        # self.robot.executePrimitive("Home()")
        self.robot.executePrimitive("MoveJ(target=-1.05 -13.19 2.11 85.09 -0.52 8.1 1.3)")

        while self.parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1":
            if self.robot.isFault():
                self.get_logger().error("Robot is at fault state")
                response.success = False
                return response
            time.sleep(1)
        response.success = True
        return response

    def fm_home_cb(self, request, response):
        self.get_logger().info(f"Incoming flipping mechanism homing request")
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response

        self.robot.executePrimitive("MoveJ(target=16.86 0.05 44.38 100.2 0.39 10.14 61.41)")

        while self.parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1":
            if self.robot.isFault():
                self.get_logger().error("Robot is at fault state")
                response.success = False
                return response
            time.sleep(1)
        response.success = True
        return response

    def move_relative_cb(self, request, response):
        self.get_logger().info(
            f"Incoming move relative request, axis: {request.direction}, magnitude: {request.magnitude}"
        )
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response
        tcp_pose = self.robot_states.tcpPose
        if request.direction == "x":
            tcp_pose[0] += request.magnitude
        elif request.direction == "y":
            tcp_pose[1] += request.magnitude
        elif request.direction == "z":
            tcp_pose[2] += request.magnitude
        quaternion_list = [
            tcp_pose[3],
            tcp_pose[4],
            tcp_pose[5],
            tcp_pose[6],
        ]
        eulerZYX_deg = self.quat2eulerZYX(quaternion_list, degree=True)
        self.robot.executePrimitive(
            f"MoveL(target={tcp_pose[0]} {tcp_pose[1]} {tcp_pose[2]} {eulerZYX_deg[0]} {eulerZYX_deg[1]} {eulerZYX_deg[2]} WORLD WORLD_ORIGIN, maxVel=0.2)"
        )
        while self.parse_pt_states(self.robot.getPrimitiveStates(), "reachedTarget") != "1":
            if self.robot.isFault():
                self.get_logger().error("Robot is at fault state")
                response.success = False
                return response
            time.sleep(1)
        response.success = True
        return response

    def contact_cb(self, request, response):
        self.get_logger().info(f"Incoming contact request")
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response
        self.get_logger().info(f"Zeroing FT Sensor")

        self.robot.executePrimitive("ZeroFTSensor()")
        time.sleep(1.0)
        self.get_logger().info(f"Contacting in progress")
        self.robot.executePrimitive("Contact()")
        while self.parse_pt_states(self.robot.getPrimitiveStates(), "primitiveName") == "Contact":
            if self.robot.isFault():
                self.get_logger().error("Robot is at fault state")
                response.success = False
                return response
            time.sleep(0.5)
        self.get_logger().info("Contact ended succesfully")
        response.success = True
        return response

    def gripper_control_cb(self, request, response):
        self.get_logger().info(f"Incoming gripper control request: {request.data}")
        if MOCK_ROBOT:
            self.get_logger().warn(f"This is a robot mock, will return True")
            response.success = True
            return response
        self.robot.writeDigitalOutput([6], [request.data])
        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    motion_server = MotionServer()
    rclpy.spin(motion_server)
    rclpy.shutdown()


if __name__ == "__main__":
    main()


# ros2 service call /move_l bin_packing_msgs/srv/MoveL '{"mode": "euler", "euler_pose": {"position": {"x": 0.7, "y": -0.1, "z": 0.3}, "orientation": {"x": 180, "y": 0, "z": 180}}}'
