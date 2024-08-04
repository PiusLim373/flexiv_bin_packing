#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import ast
from std_srvs.srv import Trigger
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
import tf2_ros
from geometry_msgs.msg import TransformStamped
from transforms3d.euler import mat2euler

class MasterController(Node):

    def __init__(self):
        super().__init__("master_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("box_config_string", "{}"),
                ("item_database_string", "{}"),
            ],
        )
        box_config_string = self.get_parameter("box_config_string").get_parameter_value().string_value
        item_database_string = self.get_parameter("item_database_string").get_parameter_value().string_value
        try:
            self.item_database = ast.literal_eval(item_database_string)
            self.box_config = ast.literal_eval(box_config_string)
            self.get_logger().info(f"Item database: {self.item_database}")
            self.get_logger().info(f"Box config: {self.box_config}")
        except (ValueError, SyntaxError) as e:
            self.get_logger().error(f"Failed to parse item_database_string: {e}")
            self.item_database = {}
            self.box_config = {}

        self.get_all_aruco_srv = self.create_client(
            GetAllAruco, "get_all_aruco", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_aruco_pose_srv = self.create_client(
            GetArucoPose, "get_aruco_pose", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.move_l_srv = self.create_client(
            MoveL, "move_l", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.set_item_srv = self.create_client(SetItem, "set_item", callback_group=MutuallyExclusiveCallbackGroup())
        self.set_box_srv = self.create_client(SetBox, "set_box", callback_group=MutuallyExclusiveCallbackGroup())
        self.pack_srv = self.create_client(Trigger, "pack", callback_group=MutuallyExclusiveCallbackGroup())
        self.reset_srv = self.create_client(Trigger, "reset", callback_group=MutuallyExclusiveCallbackGroup())
        self.get_next_placing_pose_srv = self.create_client(
            GetNextPose, "get_next_placing_pose", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)
        # if self.reset_bin_packer():
        #     self.execute()
        self.test_grip()

    def test_grip(self):
        # get a aruco pose and default config, calculate gripping pose
        self.get_logger().info("Getting the picking coordinate")
        get_aruco_pose_req = GetArucoPose.Request()
        get_aruco_pose_req.marker_id = int(21)
        future = self.get_aruco_pose_srv.call_async(get_aruco_pose_req)
        rclpy.spin_until_future_complete(self, future)
        get_aruco_pose_res = future.result()
        if get_aruco_pose_res is not None:
            if get_aruco_pose_res.success:
                mat_feature_off = self.pose_to_mat(get_aruco_pose_res.pose)
                R_x_180 = np.array([
                    [1, 0, 0, 0],
                    [0, -1, 0, 0],
                    [0, 0, -1, 0],
                    [0, 0, 0, 1]
                ])

                R_z_90 = np.array([
                    [0, -1, 0, 0],
                    [1, 0, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]
                ])
                gripper_pose =  mat_feature_off @ R_x_180 @ R_z_90
                
                translation = gripper_pose[:3, 3]
                rotation = mat2quat(gripper_pose[:3, :3])
                
                # Create a TransformStamped message
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'camera_color_optical_frame'  # Set the appropriate frame_id
                t.child_frame_id = 'gripper'
                t.transform.translation.x = translation[0]
                t.transform.translation.y = translation[1]
                t.transform.translation.z = translation[2]
                t.transform.rotation.x = rotation[1]
                t.transform.rotation.y = rotation[2]
                t.transform.rotation.z = rotation[3]
                t.transform.rotation.w = rotation[0]
                self.br.sendTransform(t)
                
                
                trans = self.tf_buffer.lookup_transform('world', 'camera_color_optical_frame', rclpy.time.Time())
                mat_trans = self.transform_to_mat(trans.transform)
                gripper_pose_world = mat_trans @ gripper_pose
                
                translation = gripper_pose_world[:3, 3]
                rotation = mat2quat(gripper_pose_world[:3, :3])
                
                rpy_rad = mat2euler(gripper_pose_world[:3, :3])
                rpy_deg = np.degrees(rpy_rad)
                
                print(rpy_deg)
                
                # Create a TransformStamped message
                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = 'world'
                t.child_frame_id = 'gripper_wrt_world'
                t.transform.translation.x = translation[0]
                t.transform.translation.y = translation[1]
                t.transform.translation.z = translation[2]
                t.transform.rotation.x = rotation[1]
                t.transform.rotation.y = rotation[2]
                t.transform.rotation.z = rotation[3]
                t.transform.rotation.w = rotation[0]
                print(t.transform.rotation)
                # Broadcast the transform
                self.br.sendTransform(t)
                
                print("i got a coor to pick")
                
                motion_req = MoveL.Request()
                motion_req.mode = MoveL.Request.QUATERNION
                motion_req.quaternion_pose.position.x = t.transform.translation.x
                motion_req.quaternion_pose.position.y = t.transform.translation.y
                motion_req.quaternion_pose.position.z = t.transform.translation.z + 0.05
                motion_req.quaternion_pose.orientation = t.transform.rotation
                future = self.move_l_srv.call_async(motion_req)
                rclpy.spin_until_future_complete(self, future)
                move_l_res = future.result()
                if move_l_res is not None:
                    if move_l_res.success:
                        print("success!")
                        exit()
            else:
                exit()    
                ''''
                bin_packing_msgs.srv.MoveL_Request(quaternion_pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.6489387549819606, y=-0.03292812383210432, z=0.06967878769167478), orientation=geometry_msgs.msg.Quaternion(x=-0.4218013808417139, y=0.9010639259941811, z=0.08356890071479017, w=0.056423711551955)), euler_pose=bin_packing_msgs.msg.EulerPose(position=geometry_msgs.msg.Point(x=0.0, y=0.0, z=0.0), orientation=geometry_msgs.msg.Vector3(x=0.0, y=0.0, z=0.0)), mode='quaternion')
                ros2 service call /move_l bin_packing_msgs/srv/MoveL '{"mode": "quaternion", "quaternion_pose": {"position": {"x": 0.6489387549819606, "y": -0.03292812383210432, "z": 0.1}, "orientation": {"x": -0.4218013808417139, "y": 0.9010639259941811, "z": 0.08356890071479017, "w": 0.056423711551955}}}'
                '''

    def pose_to_mat(self, pose):
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        mat = np.eye(4)
        mat[:3, :3] = quat2mat(quat)
        mat[:3, 3] = pos
        return mat
    
    def transform_to_mat(self, transform):
        '''Convert a Transform message to a 4x4 numpy matrix.'''
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]
        mat = np.eye(4)
        mat[:3, :3] = quat2mat(rotation)
        mat[:3, 3] = translation
        return mat

    def reset_bin_packer(self):
        print("calling reset")
        req = Trigger.Request()
        future = self.reset_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2)
        reset_res = future.result()
        if reset_res is not None:
            if reset_res.success:
                print("successfully reset the bin packer")
                return True
        print("Error when resetting bin packer")
        return False

    def execute(self):
        try:
            # call to get all available aruco
            get_all_aruco_req = GetAllAruco.Request()
            future = self.get_all_aruco_srv.call_async(get_all_aruco_req)
            rclpy.spin_until_future_complete(self, future)
            get_all_aruco_res = future.result()
            if get_all_aruco_res is not None:
                if get_all_aruco_res.success:

                    # setting box
                    self.get_logger().info(f"Starting to set box in bin packer")

                    set_box_req = SetBox.Request()
                    set_box_req.box.name = self.box_config["name"]
                    set_box_req.box.length = round(self.box_config["dimension"]["length"], 3)
                    set_box_req.box.width = round(self.box_config["dimension"]["width"], 3)
                    set_box_req.box.height = round(self.box_config["dimension"]["height"], 3)
                    set_box_req.pose.position.x = self.box_config["pose"]["position"]["x"]
                    set_box_req.pose.position.y = self.box_config["pose"]["position"]["y"]
                    set_box_req.pose.position.z = self.box_config["pose"]["position"]["z"]
                    set_box_req.pose.orientation.x = self.box_config["pose"]["orientation"]["x"]
                    set_box_req.pose.orientation.y = self.box_config["pose"]["orientation"]["y"]
                    set_box_req.pose.orientation.z = self.box_config["pose"]["orientation"]["z"]
                    set_box_req.pose.orientation.w = self.box_config["pose"]["orientation"]["w"]
                    print(set_box_req)
                    future = self.set_box_srv.call_async(set_box_req)
                    rclpy.spin_until_future_complete(self, future)
                    set_box_res = future.result()
                    if set_box_res is not None:
                        if not set_box_res.success:
                            self.get_logger().error(f"error encountered when setting box in bin packer")
                            return
                    else:
                        raise Exception("set box rosservice future not returning")

                    # for each aruco, get the default lhw item dimension and add to bin packing machine
                    self.all_aruco_markers = get_all_aruco_res.aruco_markers
                    self.get_logger().info(
                        f"All aruco presense: {self.all_aruco_markers}, starting to set item in bin packer"
                    )
                    for aruco_marker_id in self.all_aruco_markers:
                        item_config = self.item_database.get(str(aruco_marker_id), None)
                        if item_config:
                            print(f"item with marker id {aruco_marker_id} found in database")
                            print(item_config)
                            set_item_req = SetItem.Request()
                            set_item_req.item_id = str(aruco_marker_id)
                            set_item_req.item.name = f"Arcuco id {str(aruco_marker_id)}"
                            set_item_req.item.length = item_config["dimension"]["length"]
                            set_item_req.item.width = item_config["dimension"]["width"]
                            set_item_req.item.height = item_config["dimension"]["height"]
                            set_item_req.item.color = item_config["color"]
                            future = self.set_item_srv.call_async(set_item_req)
                            rclpy.spin_until_future_complete(self, future)
                            set_item_res = future.result()
                            if set_item_res is not None:
                                if not set_item_res.success:
                                    self.get_logger().error(f"error encountered when setting item in bin packer")
                                    return
                            else:
                                raise Exception("set item rosservice future not returning")

                        else:
                            print(
                                f"item with marker id {aruco_marker_id} not found in database, will skip processing it"
                            )
                    # all item added, calling pack service
                    self.get_logger().info(f"All items have been sent to bin packer, calling pack service")
                    pack_req = Trigger.Request()
                    future = self.pack_srv.call_async(pack_req)
                    rclpy.spin_until_future_complete(self, future)
                    pack_res = future.result()
                    if pack_res is not None:
                        if not pack_res.success:
                            self.get_logger().error(f"error encountered when request to pack item in bin packer")
                            return
                    else:
                        raise Exception("set item rosservice future not returning")

                    self.get_logger().info(f"Packing done, calling to get next placing pose")

                    has_transferred_all_item = False
                    while not has_transferred_all_item:
                        get_next_placing_pose_req = GetNextPose.Request()
                        future = self.get_next_placing_pose_srv.call_async(get_next_placing_pose_req)
                        rclpy.spin_until_future_complete(self, future)
                        get_next_placing_pose_res = future.result()
                        if get_next_placing_pose_res is not None:
                            if get_next_placing_pose_res.output == GetNextPose.Response.COMPLETED:
                                self.get_logger().info(f"All items has been packed to box, the flow has completed!")
                                has_transferred_all_item = True

                            elif get_next_placing_pose_res.output == GetNextPose.Response.SUCCESS:
                                self.current_item_to_transfer = {
                                    "item_id": get_next_placing_pose_res.item_id,
                                    "rotation_config": get_next_placing_pose_res.rotation_config,
                                    "placing_pose": get_next_placing_pose_res.placing_pose,
                                    "picking_coor": None,
                                    "flipping_mechanism": False,
                                    "offset_before_flip": False,
                                    "offset_after_flip": False,
                                }
                                self.get_logger().info(
                                    f"Will transfer item with marker id {self.current_item_to_transfer['item_id']} to ({self.current_item_to_transfer['placing_pose'].position.x}, {self.current_item_to_transfer['placing_pose'].position.y}, {self.current_item_to_transfer['placing_pose'].position.z}) with rotation config {self.current_item_to_transfer['rotation_config']}"
                                )

                                # get picking coor
                                self.get_logger().info("Getting the picking coordinate")
                                get_aruco_pose_req = GetArucoPose.Request()
                                get_aruco_pose_req.marker_id = int(self.current_item_to_transfer["item_id"])
                                future = self.get_aruco_pose_srv.call_async(get_aruco_pose_req)
                                rclpy.spin_until_future_complete(self, future)
                                get_aruco_pose_res = future.result()
                                if get_aruco_pose_res is not None:
                                    if get_aruco_pose_res.success:
                                        self.current_item_to_transfer["placing_pose"] = get_aruco_pose_res.pose
                                        if self.current_item_to_transfer["rotation_config"] == "LWH":
                                            # default case, transform only the top down z
                                            pass
                                        elif self.current_item_to_transfer["rotation_config"] == "WLH":
                                            # transform top down z + 90deg offset
                                            self.current_item_to_transfer["offset_before_flip"] = True
                                        elif self.current_item_to_transfer["rotation_config"] == "HWL":
                                            # grip with LWH, then transder to flipping mechanism
                                            self.current_item_to_transfer["flipping_mechanism"] = True
                                        elif self.current_item_to_transfer["rotation_config"] == "HLW":
                                            # grip with WLH, then transder to flipping mechanism
                                            self.current_item_to_transfer["offset_before_flip"] = True
                                            self.current_item_to_transfer["flipping_mechanism"] = True
                                        elif self.current_item_to_transfer["rotation_config"] == "WHL":
                                            # grip with HWL, then transder to flipping mechanism, offset after flip
                                            self.current_item_to_transfer["flipping_mechanism"] = True
                                            self.current_item_to_transfer["offset_after_flip"] = True
                                        elif self.current_item_to_transfer["rotation_config"] == "LHW":
                                            # grip with HLW, then transder to flipping mechanism, offset after flip
                                            self.current_item_to_transfer["offset_before_flip"] = True
                                            self.current_item_to_transfer["flipping_mechanism"] = True
                                            self.current_item_to_transfer["offset_after_flip"] = True

                                        # create picking task
                                        self.pick(self.current_item_to_transfer)
                                else:
                                    raise Exception("get aruco pose rosservice future not returning")

                        else:
                            raise Exception("get next placing coor rosservice future not returning")

                else:
                    self.get_logger().error(f"error getting aruco or no aruco is presense")
            else:
                raise Exception("get all aruco rosservice future not returning")
        except Exception as e:
            self.get_logger().error(f"Master controller execute error with msg: {e}")

    def pick(self, item_config):
        self.get_logger().info(f"picking item {item_config}")
        # using the item config, do transformation here and create service call to motion server
        input("press enter to continue")
        return True


def main(args=None):
    rclpy.init(args=args)
    master_controller = MasterController()
    rclpy.spin(master_controller)
    rclpy.shutdown()
    # executor = rclpy.executors.MultiThreadedExecutor()
    # executor.add_node(master_controller)
    # executor.spin()
    # executor.shutdown()
    # master_controller.destory_node()
    # rclpy.shutdown()


if __name__ == "__main__":
    main()
