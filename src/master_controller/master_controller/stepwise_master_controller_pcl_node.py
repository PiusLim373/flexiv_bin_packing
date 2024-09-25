#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import ast
from std_srvs.srv import Trigger, SetBool
import numpy as np
from transforms3d.quaternions import quat2mat, mat2quat
import tf2_ros
from geometry_msgs.msg import TransformStamped, Pose, Point, Quaternion
from transforms3d.euler import mat2euler, euler2quat
import transforms3d
import math
import time
from tf2_geometry_msgs import do_transform_pose
import tf_transformations
from icecream import ic
import copy
import requests
import random
ic.configureOutput(includeContext=True)


class MasterController(Node):

    def __init__(self):
        super().__init__("master_controller")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("box_config_string", "{}"),
                ("flipping_mechanism_pose_string", "{}"),
                ("flipping_mechanism_endpoint", "")
            ],
        )
        box_config_string = self.get_parameter("box_config_string").get_parameter_value().string_value
        flipping_mechanism_pose_string = (
            self.get_parameter("flipping_mechanism_pose_string").get_parameter_value().string_value
        )
        self.flipping_mechanism_endpoint = self.get_parameter("flipping_mechanism_endpoint").get_parameter_value().string_value
        
        try:
            self.box_config = ast.literal_eval(box_config_string)
            self.flipping_mechanism_config = ast.literal_eval(flipping_mechanism_pose_string)
            self.flipping_mechanism_pose = Pose()
            self.flipping_mechanism_pose.position.x = self.flipping_mechanism_config["position"]["x"]
            self.flipping_mechanism_pose.position.y = self.flipping_mechanism_config["position"]["y"]
            self.flipping_mechanism_pose.position.z = self.flipping_mechanism_config["position"]["z"]
            self.flipping_mechanism_pose.orientation.x = self.flipping_mechanism_config["orientation"]["x"]
            self.flipping_mechanism_pose.orientation.y = self.flipping_mechanism_config["orientation"]["y"]
            self.flipping_mechanism_pose.orientation.z = self.flipping_mechanism_config["orientation"]["z"]
            self.flipping_mechanism_pose.orientation.w = self.flipping_mechanism_config["orientation"]["w"]
            self.get_logger().info(f"Box config: {self.box_config}")
            self.get_logger().info(f"Flipping Mechanism pose: {self.flipping_mechanism_pose}")
            self.get_logger().info(f"Flipping Mechanism endpoint: {self.flipping_mechanism_endpoint}")
        except (ValueError, SyntaxError) as e:
            self.get_logger().error(f"Failed to parse item_database_string: {e}")
            self.box_config = {}
            self.flipping_mechanism_config = {}

        # variables:
        self.current_item_to_transfer = None

        # service proxy
        self.get_all_box_detail_srv = self.create_client(
            GetAllBoxDetail, "get_all_box_detail", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.get_fm_box_detail_srv = self.create_client(
            GetFMBoxDetail, "get_fm_box_detail", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.move_l_srv = self.create_client(MoveL, "move_l", callback_group=MutuallyExclusiveCallbackGroup())
        self.move_relative_srv = self.create_client(
            MoveRelative, "move_relative", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.contact_srv = self.create_client(Trigger, "contact", callback_group=MutuallyExclusiveCallbackGroup())
        self.gripper_control_srv = self.create_client(
            SetBool, "gripper_control", callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.home_robot_srv = self.create_client(Trigger, "home", callback_group=MutuallyExclusiveCallbackGroup())
        self.fm_home_robot_srv = self.create_client(Trigger, "fm_home", callback_group=MutuallyExclusiveCallbackGroup())
        self.set_item_srv = self.create_client(SetItem, "set_item", callback_group=MutuallyExclusiveCallbackGroup())
        self.set_box_srv = self.create_client(SetBox, "set_box", callback_group=MutuallyExclusiveCallbackGroup())
        self.pack_srv = self.create_client(Trigger, "pack", callback_group=MutuallyExclusiveCallbackGroup())
        self.reset_srv = self.create_client(Trigger, "reset", callback_group=MutuallyExclusiveCallbackGroup())
        self.get_next_placing_pose_srv = self.create_client(
            GetNextPose, "get_next_placing_pose", callback_group=MutuallyExclusiveCallbackGroup()
        )

        # services
        self.create_service(
            Trigger, "get_all_box_and_pack", self.get_all_box_and_pack_cb
        )  # a service that activate camera and get all available marker
        self.create_service(
            Trigger, "get_next_coordinate", self.get_next_coordinate_cb
        )  # a service that populate data for item to be transder
        self.create_service(
            TransferItem, "transfer_item", self.transfer_item_cb
        )  # a service that pick the item based on rotational configuration set
        self.create_service(
            Trigger, "transfer_from_flipping_mechanism", self.transfer_from_flipping_mechanism_cb
        )  # a service that transfer item from fm based on rotational configuration set
        self.create_service(
            Trigger, "clear_halfway_item", self.clear_halfway_item_cb
        )  # a service that place the item in the box

        # transforms related
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.br = tf2_ros.TransformBroadcaster(self)

        # main function to run below
        self.reset_bin_packer()
        self.setup_box()

        # for bypassing
        '''
        self.all_charuco_markers = [0]
        placing_pose = Pose()
        placing_pose.position.x = 0.3225
        placing_pose.position.y = -0.375
        placing_pose.position.z = 0.092
        placing_pose.orientation.x = 0.0
        placing_pose.orientation.y = -1.0
        placing_pose.orientation.z = 0.0
        placing_pose.orientation.w = 0.0
        
        picking_pose = Pose()  # depends on camera
        picking_pose.position.x = 0.6
        picking_pose.position.y = -0.114
        picking_pose.position.z = 0.1
        picking_pose.orientation.x = 0.051
        picking_pose.orientation.y = 0.999
        picking_pose.orientation.z = 0.002
        picking_pose.orientation.w = 0.000

        self.current_item_to_transfer = {
            "item_id": 0,
            "rotation_config": "LHW",
            "placing_pose": placing_pose,
            "item_pose": picking_pose,
            "flipping_mechanism": True,
            "offset_before_flip": True,
            "offset_after_flip": False,
            "dimension": {'length': 0.145, 'width': 0.092, 'height': 0.05},
        }
        '''
        self.get_logger().info("Stepwise master controller ready, please call services to continue")

    def pose_to_mat(self, pose):
        quat = [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z]
        pos = np.array([pose.position.x, pose.position.y, pose.position.z])
        mat = np.eye(4)
        mat[:3, :3] = quat2mat(quat)
        mat[:3, 3] = pos
        return mat

    def transform_to_mat(self, transform):
        """Convert a Transform message to a 4x4 numpy matrix."""
        translation = [transform.translation.x, transform.translation.y, transform.translation.z]
        rotation = [transform.rotation.w, transform.rotation.x, transform.rotation.y, transform.rotation.z]
        mat = np.eye(4)
        mat[:3, :3] = quat2mat(rotation)
        mat[:3, 3] = translation
        return mat

    def reset_bin_packer(self):
        req = Trigger.Request()
        future = self.reset_srv.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2)
        reset_res = future.result()
        if reset_res is not None:
            if reset_res.success:
                self.get_logger().info("successfully reset the bin packer")
                return True
        self.get_logger().error("Error when resetting bin packer")
        return False

    def setup_box(self):
        self.get_logger().info("Setting up box")
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
        ic(set_box_req)
        future = self.set_box_srv.call_async(set_box_req)
        rclpy.spin_until_future_complete(self, future)
        set_box_res = future.result()
        if set_box_res is not None:
            if set_box_res.success:
                self.get_logger().info(f"Successfully setup Box")
            else:
                self.get_logger().error(f"error encountered when setting box in bin packer")
        else:
            raise Exception("set box rosservice future not returning")

    def get_all_box_and_pack_cb(self, request, response):
        self.get_logger().info("Got request to get all ar marker")
        get_all_box_detail_req = GetAllBoxDetail.Request()
        get_all_box_detail_res = self.get_all_charuco_srv.call(get_all_box_detail_req)
        if get_all_box_detail_res is not None:
            if get_all_box_detail_res.success:
                self.get_logger().info(f"{len(get_all_box_detail_res.boxes)} items detected")
                for item in get_all_box_detail_res.boxes:
                    ic(item)
                    set_item_req = SetItem.Request()
                    set_item_req.item_id = item.shape.name
                    set_item_req.item.name = item.shape.name
                    set_item_req.item.length = item.shape.length
                    set_item_req.item.width = item.shape.width
                    set_item_req.item.height = item.shape.height
                    set_item_req.item.color = random.choice(["red", "blue", "green", "purple", "yellow", "orange"])
                    set_item_res = self.set_item_srv.call(set_item_req)
                    if set_item_res is not None:
                        if not set_item_res.success:
                            self.get_logger().error(f"error encountered when setting item in bin packer")
                            response.success = False
                # all item added, calling pack service
                self.get_logger().info(f"All items have been sent to bin packer, calling pack service")
                pack_req = Trigger.Request()
                pack_res = self.pack_srv.call(pack_req)
                if pack_res is not None:
                    if pack_res.success:
                        self.get_logger().info("Done packing items in bin")
                        response.success = True
                    else:
                        self.get_logger().error(f"error encountered when request to pack item in bin packer")
                        response.success = False
                else:
                    raise Exception("set item rosservice future not returning")
                    response.success = False
            else:
                self.get_logger().error(f"error encountered when getting all marker")
                response.success = False
        else:
            raise Exception("get all charuco rosservice future not returning")
            response.success = False
        return response

    def get_next_coordinate_cb(self, request, response):
        # only call next get next pose if there isnt residue current item to transfer
        ic(self.current_item_to_transfer)
        if self.current_item_to_transfer == None:
            get_next_placing_pose_req = GetNextPose.Request()
            get_next_placing_pose_res = self.get_next_placing_pose_srv.call(get_next_placing_pose_req)
            if get_next_placing_pose_res is not None:
                if get_next_placing_pose_res.output == GetNextPose.Response.COMPLETED:
                    self.get_logger().info(f"All items has been packed to box, the flow has completed!")
                    response.success = True
                    return response

                elif get_next_placing_pose_res.output == GetNextPose.Response.SUCCESS:
                    self.current_item_to_transfer = {
                        "item_id": get_next_placing_pose_res.item_id,
                        "rotation_config": get_next_placing_pose_res.rotation_config,
                        "placing_pose": get_next_placing_pose_res.placing_pose,
                        "picking_pose": None,
                        "item_pose": None,
                        "flipping_mechanism": False,
                        "offset_before_flip": False,
                        "offset_after_flip": False,
                        "dimension": {"length": get_next_placing_pose_res.shape.length, "width": get_next_placing_pose_res.shape.width, "height": get_next_placing_pose_res.shape.height},
                    }
        else:
            self.get_logger().warn("Item transferred halfway detected")

        self.get_logger().info(
            f"Will transfer item with marker id {self.current_item_to_transfer['item_id']} to ({self.current_item_to_transfer['placing_pose'].position.x}, {self.current_item_to_transfer['placing_pose'].position.y}, {self.current_item_to_transfer['placing_pose'].position.z}) with rotation config {self.current_item_to_transfer['rotation_config']}"
        )
        # get picking coor
        self.get_logger().info("Getting the picking coordinate")
        get_charuco_pose_req = GetChArUcoPose.Request()
        get_charuco_pose_req.charuco_id = int(self.current_item_to_transfer["item_id"])
        get_charuco_pose_res = self.get_charuco_pose_srv.call(get_charuco_pose_req)
        if get_charuco_pose_res.success:
            charuco_in_world = self.transform_cam_charuco_to_world("camera_color_optical_frame", get_charuco_pose_res.pose)

            # rotate about x by 180deg and z by 90deg to convert from arcuo frame to gripper frame
            # R_x_180 = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
            R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            # temp = self.transform_pose(charuco_in_world, R_x_180)
            self.current_item_to_transfer["item_pose"] = self.transform_pose(charuco_in_world, R_z_90)

            self.publish_tf("world", "item_pose", self.current_item_to_transfer["item_pose"])

            if self.current_item_to_transfer["rotation_config"] == "WLH":
                # default case, transform only the top down z
                pass
            
            elif self.current_item_to_transfer["rotation_config"] == "LWH":
                # transform top down z + 90deg offset
                self.current_item_to_transfer["offset_before_flip"] = True
            
            elif self.current_item_to_transfer["rotation_config"] == "WHL":
                # grip with WLH, then transder to flipping mechanism
                self.current_item_to_transfer["flipping_mechanism"] = True
                
            elif self.current_item_to_transfer["rotation_config"] == "HWL":
                # grip with WLH, then transder to flipping mechanism, offset after flip
                self.current_item_to_transfer["flipping_mechanism"] = True
                self.current_item_to_transfer["offset_after_flip"] = True
                
            elif self.current_item_to_transfer["rotation_config"] == "LHW":
                # grip with LWH, then transder to flipping mechanism
                self.current_item_to_transfer["offset_before_flip"] = True
                self.current_item_to_transfer["flipping_mechanism"] = True
                
            elif self.current_item_to_transfer["rotation_config"] == "HLW":
                # grip with LWH, then transder to flipping mechanism, offset after flip
                self.current_item_to_transfer["offset_before_flip"] = True
                self.current_item_to_transfer["flipping_mechanism"] = True
                self.current_item_to_transfer["offset_after_flip"] = True

            self.get_logger().info("successful populating next item configs")
            ic(self.current_item_to_transfer)
            response.success = True
        else:
            self.get_logger().error("Error getting 3D data from pixel coordinate")
            response.success = False
        return response

    def transfer_item_cb(self, request, response):
        if self.current_item_to_transfer == None:
            self.get_logger().error("Invalid item to process")
            response.outcome = TransferItem.Response.ERROR
            return response
        _, _, yaw = tf_transformations.euler_from_quaternion(
            [
                self.current_item_to_transfer["item_pose"].orientation.x,
                self.current_item_to_transfer["item_pose"].orientation.y,
                self.current_item_to_transfer["item_pose"].orientation.z,
                self.current_item_to_transfer["item_pose"].orientation.w,
            ]
        )
        ic(math.degrees(yaw))
        if self.current_item_to_transfer["offset_before_flip"]:
            if math.degrees(yaw) >= 0:
                R_z_270 = np.array([[0, 1, 0, 0], [-1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self.current_item_to_transfer["picking_pose"] = self.transform_pose(
                    self.current_item_to_transfer["item_pose"], R_z_270
                )
            else:
                R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self.current_item_to_transfer["picking_pose"] = self.transform_pose(
                    self.current_item_to_transfer["item_pose"], R_z_90
                )
            ic(self.current_item_to_transfer["picking_pose"])
        else:
            if abs(math.degrees(yaw)) < 90:
                R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                self.current_item_to_transfer["picking_pose"] = self.transform_pose(
                    self.current_item_to_transfer["item_pose"], R_z_180
                )
            else:
                self.current_item_to_transfer["picking_pose"] = self.current_item_to_transfer["item_pose"]

        self.publish_tf("world", "picking_pose", self.current_item_to_transfer["picking_pose"])
        if not self.pick(copy.deepcopy(self.current_item_to_transfer["picking_pose"])):
            self.housekeeping()
            response.outcome = TransferItem.Response.ERROR
            return response

        if self.current_item_to_transfer["flipping_mechanism"]:
            self.get_logger().error("Flipping Mechanism is needed, transferring here instead")
            self.publish_tf("world", "fm_placing_pose_ori", self.flipping_mechanism_pose)
            offset_placing_pose = copy.deepcopy(self.flipping_mechanism_pose)
            offset_placing_pose.pose.position.z -= self.current_item_to_transfer["dimension"]["height"]
            if self.current_item_to_transfer["offset_before_flip"]:
                offset_placing_pose.position.y += (0.5 * self.current_item_to_transfer["dimension"]["width"] + 0.01)
            else:
                offset_placing_pose.position.y += (0.5 * self.current_item_to_transfer["dimension"]["length"] + 0.01)
            self.publish_tf("world", "fm_placing_pose", offset_placing_pose)
            if not self.place(offset_placing_pose):
                self.housekeeping()
                response.outcome = TransferItem.Response.ERROR
                return response
            self.get_logger().error("Item transferred successfully, calling for flip")
            
            input("Press Enter to continue...")
            requests.get(self.flipping_mechanism_endpoint)
            # need to feedback to caller that this item is transferred to FM, need to pick from there later
            response.outcome = TransferItem.Response.FLIPPING_MECHANISM
            return response

        else:
            self.publish_tf("world", "placing_pose", self.current_item_to_transfer["placing_pose"])
            if not self.place(copy.deepcopy(self.current_item_to_transfer["placing_pose"])):
                self.housekeeping()
                response.outcome = TransferItem.Response.ERROR
                return response

            self.get_logger().error("Item transferred successfully!")
            self.current_item_to_transfer = None
            response.outcome = TransferItem.Response.BOX
            return response
        
    def transfer_from_flipping_mechanism_cb(self, request, response):
        self.get_logger().info("Got request to transfer item from flipping mechanism")
        if self.current_item_to_transfer == None:
            self.get_logger().error("Invalid item to process")
            response.success = False
            return response
        req = GetFMBoxDetail.Request()
        get_fm_box_detail_res = self.get_fm_box_detail_srv.call(req)
        self.get_logger().info("Successfully get flipping mechanism box detail")
        if get_fm_box_detail_res.success:
            fm_picking_pose = self.transform_cam_charuco_to_world("camera_depth_optical_frame", get_fm_box_detail_res.box.pose)
            yaw = get_fm_box_detail_res.box.yaw
            ic(yaw)
            if self.current_item_to_transfer["rotation_config"] == "WHL":
                if self.current_item_to_transfer["dimension"]["width"] > self.current_item_to_transfer["dimension"]["height"]:
                    # detected x axis is W, align gripper x to detected x
                    self.get_logger().info("Rotation configuration WHL, Detected x axis is W, align gripper x to detected x")
                    if yaw < 0:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                else:
                    # detected x axis is H, post rotation is neeeded
                    self.get_logger().info("Rotation configuration WHL, Detected x axis is H, post rotation is neeeded")
                    R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_90)
                    if yaw < -90 or yaw > 90:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                    
            elif self.current_item_to_transfer["rotation_config"] == "HWL":
                if self.current_item_to_transfer["dimension"]["height"] > self.current_item_to_transfer["dimension"]["width"]:
                    # detected x axis is H, align gripper x to detected x
                    self.get_logger().info("Rotation configuration HWL, Detected x axis is H, align gripper x to detected x")
                    if yaw < 0:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                else:
                    # detected x axis is W, post rotation is neeeded
                    self.get_logger().info("Rotation configuration HWL, Detected x axis is W, post rotation is neeeded")
                    R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_90)
                    if yaw < -90 or yaw > 90:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                        
            elif self.current_item_to_transfer["rotation_config"] == "LHW":
                if self.current_item_to_transfer["dimension"]["length"] > self.current_item_to_transfer["dimension"]["height"]:
                    # detected x axis is H, align gripper x to detected x
                    self.get_logger().info("Rotation configuration LHW, Detected x axis is L, align gripper x to detected x")
                    if yaw < 0:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                else:
                    # detected x axis is W, post rotation is neeeded
                    self.get_logger().info("Rotation configuration LHW, Detected x axis is H, post rotation is neeeded")
                    R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_90)
                    if yaw < -90 or yaw > 90:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                        
            elif self.current_item_to_transfer["rotation_config"] == "HLW":
                if self.current_item_to_transfer["dimension"]["height"] > self.current_item_to_transfer["dimension"]["length"]:
                    # detected x axis is H, align gripper x to detected x
                    self.get_logger().info("Rotation configuration HLW, Detected x axis is H, align gripper x to detected x")
                    if yaw < 0:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
                else:
                    # detected x axis is W, post rotation is neeeded
                    self.get_logger().info("Rotation configuration HLW, Detected x axis is L, post rotation is neeeded")
                    R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                    fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_90)
                    if yaw < -90 or yaw > 90:
                        self.get_logger().info("Rotation correction is neeeded")
                        R_z_180 = np.array([[-1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
                        fm_picking_pose = self.transform_pose(fm_picking_pose, R_z_180)
            
            self.publish_tf("world", "picking_pose", fm_picking_pose)
            if not self.pick(fm_picking_pose):
                self.housekeeping()
                response.success = False
                return response
            
            self.publish_tf("world", "placing_pose", self.current_item_to_transfer["placing_pose"])
            if not self.place(copy.deepcopy(self.current_item_to_transfer["placing_pose"])):
                self.housekeeping()
                response.success = False
                return response

            self.get_logger().info("Item transferred from FM successfully!")
            self.current_item_to_transfer = None
            response.success = True
            return response
            
        else:
            self.get_logger().error("Error getting flipping mechanism box detail")
            response.success = False
            return response

    def clear_halfway_item_cb(self, request, response):
        self.get_logger().error("Reqest to clear halfway item")
        self.current_item_to_transfer = None
        return response

    def housekeeping(self):
        pass

    def transform_cam_charuco_to_world(self, camera_frame, pose):
        transform = self.tf_buffer.lookup_transform("world", camera_frame, rclpy.time.Time())
        pose_in_world_frame = do_transform_pose(pose, transform)
        return pose_in_world_frame

    def apply_gripper_frame_transform(self, pose):
        pose_t_matrix = self.pose_to_mat(pose)
        R_x_180 = np.array([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
        R_z_90 = np.array([[0, -1, 0, 0], [1, 0, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
        pose_t_matrix_after_gripper = pose_t_matrix @ R_x_180 @ R_z_90
        position = pose_t_matrix_after_gripper[:3, 3]
        quaternion = tf_transformations.quaternion_from_matrix(pose_t_matrix_after_gripper)
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]
        return pose

    def transform_pose(self, pose, transform):
        pose_t_matrix = self.pose_to_mat(pose)
        pose_t_matrix_after_transform = pose_t_matrix @ transform
        position = pose_t_matrix_after_transform[:3, 3]
        quaternion = tf_transformations.quaternion_from_matrix(pose_t_matrix_after_transform)
        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quaternion[0]
        pose.orientation.y = quaternion[1]
        pose.orientation.z = quaternion[2]
        pose.orientation.w = quaternion[3]

        return pose

    def publish_tf(self, parent, frame_name, pose):
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = parent  # Parent frame
        transform_stamped.child_frame_id = frame_name  # Child frame
        transform_stamped.transform.translation.x = pose.position.x
        transform_stamped.transform.translation.y = pose.position.y
        transform_stamped.transform.translation.z = pose.position.z
        transform_stamped.transform.rotation = pose.orientation
        self.br.sendTransform(transform_stamped)

    def pick(self, pose, is_fm=False):
        move_l_req = MoveL.Request()
        move_l_req.mode = MoveL.Request.QUATERNION
        move_l_req.quaternion_pose = pose
        move_l_req.quaternion_pose.position.z += 0.03
        move_res = self.move_l_srv.call(move_l_req)
        if not move_res.success:
            self.get_logger().warn("Move L failed")
            return False

        # contact with item
        contact_req = Trigger.Request()
        contact_res = self.contact_srv.call(contact_req)
        if not contact_res.success:
            self.get_logger().warn("Contact failed")
            return False

        # activate gripper
        self.get_logger().info("Activating Gripper")
        gripper_req = SetBool.Request()
        gripper_req.data = True
        gripper_control_res = self.gripper_control_srv.call(gripper_req)
        if not gripper_control_res.success:
            self.get_logger().warn("Gripper control failed")
            return False

        time.sleep(1)

        # z up 5cm
        move_relative_req = MoveRelative.Request()
        move_relative_req.direction = "z"
        move_relative_req.magnitude = 0.045
        move_res = self.move_relative_srv.call(move_relative_req)
        if not move_res.success:
            self.get_logger().warn("Move Relative failed")
            return False

        # homing
        if is_fm:
            move_res = self.fm_home_robot_srv.call(Trigger.Request())
            if not move_res.success:
                self.get_logger().warn("Homing failed")
                return False
        else:
            move_res = self.home_robot_srv.call(Trigger.Request())
            if not move_res.success:
                self.get_logger().warn("Homing failed")
                return False

        return True

    def place(self, pose, is_fm=False):
        # go to 5cm above placing coor
        move_l_req = MoveL.Request()
        move_l_req.mode = MoveL.Request.QUATERNION
        move_l_req.quaternion_pose = pose
        move_l_req.quaternion_pose.position.z += 0.1
        move_res = self.move_l_srv.call(move_l_req)
        if not move_res.success:
            self.get_logger().warn("Move L failed")
            return False

        # z down 5cm
        move_relative_req = MoveRelative.Request()
        move_relative_req.direction = "z"
        move_relative_req.magnitude = -0.095
        move_res = self.move_relative_srv.call(move_relative_req)
        if not move_res.success:
            self.get_logger().warn("Move Relative failed")
            return False

        # deactivate gripper
        self.get_logger().info("Deactivating Gripper")
        gripper_req = SetBool.Request()
        gripper_req.data = False
        gripper_control_res = self.gripper_control_srv.call(gripper_req)
        if not gripper_control_res.success:
            self.get_logger().warn("Gripper control failed")
            return False

        time.sleep(1)

        # z up 5cm
        move_relative_req = MoveRelative.Request()
        move_relative_req.direction = "z"
        move_relative_req.magnitude = 0.095
        move_res = self.move_relative_srv.call(move_relative_req)
        if not move_res.success:
            self.get_logger().warn("Move Relative failed")
            return False

        # homing
        if is_fm:
            move_res = self.fm_home_robot_srv.call(Trigger.Request())
            if not move_res.success:
                self.get_logger().warn("Homing failed")
                return False
        else:
            move_res = self.home_robot_srv.call(Trigger.Request())
            if not move_res.success:
                self.get_logger().warn("Homing failed")
                return False

        return True


def main(args=None):
    rclpy.init(args=args)
    master_controller = MasterController()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(master_controller)
    executor.spin()
    executor.shutdown()
    master_controller.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
