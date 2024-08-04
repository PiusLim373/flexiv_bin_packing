#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from std_srvs.srv import *
from geometry_msgs.msg import Pose
import random
import threading
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as anim
from mpl_toolkits.mplot3d import Axes3D

NUMBER_OF_DECIMAL = 3
ROTATION = ["LWH", "WLH"]
STRIDE = ["x", "y", "z"]


class Box:
    def __init__(self, length, width, height, pose, name="box"):
        self.length = round(length, NUMBER_OF_DECIMAL)
        self.width = round(width, NUMBER_OF_DECIMAL)
        self.height = round(height, NUMBER_OF_DECIMAL)
        self.name = name
        self.items_in_box = []
        self.pose = pose

    # intersection check
    def has_intersection(self, inter_check_coor, inter_check_dim, placing_coor, item_dim):
        inter_check_x, inter_check_y, inter_check_z = inter_check_coor
        inter_check_length, inter_check_width, inter_check_height = inter_check_dim
        placing_x, placing_y, placing_z = placing_coor
        item_length, item_width, item_height = item_dim

        x_intersect = (inter_check_x < placing_x + item_length) and (inter_check_x + inter_check_length > placing_x)
        y_intersect = (inter_check_y < placing_y + item_width) and (inter_check_y + inter_check_width > placing_y)
        z_intersect = (inter_check_z < placing_z + item_height) and (inter_check_z + inter_check_height > placing_z)
        return x_intersect and y_intersect and z_intersect

    # box limit check
    def has_exceeded_box_limit(self, placing_coor, item_dimension):
        if (
            (placing_coor[0] + item_dimension[0]) > self.length
            or (placing_coor[1] + item_dimension[1]) > self.width
            or (placing_coor[2] + item_dimension[2]) > self.height
        ):
            return True
        return False

    def get_all_z_placing_coor(self, data):
        x, y, z = data
        return_data = []
        step = 10 ** (-NUMBER_OF_DECIMAL)
        num_steps = int(round(z / step)) + 1
        for i in range(num_steps):
            current_z = i * step
            yield (x, y, round(current_z, NUMBER_OF_DECIMAL))

    def reset(self):
        self.items_in_box = []

    # main packing function
    def put_item(self, item, consider_gravity):
        # box still empty, just put at origin
        if len(self.items_in_box) == 0:
            for rotation_type in item.available_rotations:
                item.set_rotation_type(rotation_type)
                if self.has_exceeded_box_limit((0, 0, 0), item.get_rotated_dimension()):
                    print(f"exceeded box limit for item {item.item_id}")
                    continue
                self.items_in_box.append(item)
                return (True, (0, 0, 0))
            return (False, (0, 0, 0))
        else:
            # try offset along diff axis (x, y, and z)
            for stride_dir in STRIDE:
                for item_in_box in self.items_in_box:
                    item_in_box_dim = item_in_box.get_rotated_dimension()
                    item_in_box_coor = item_in_box.coor

                    # try diff rotation, starting with the one with largest surface area
                    for rotation_type in item.available_rotations:

                        item.set_rotation_type(rotation_type)
                        item_dim = item.get_rotated_dimension()
                        if stride_dir == "x":
                            placing_coor = (
                                item_in_box_coor[0] + item_in_box_dim[0],
                                item_in_box_coor[1],
                                item_in_box_coor[2],
                            )
                        elif stride_dir == "y":
                            placing_coor = (
                                item_in_box_coor[0],
                                item_in_box_coor[1] + item_in_box_dim[1],
                                item_in_box_coor[2],
                            )
                        elif stride_dir == "z":
                            placing_coor = (
                                item_in_box_coor[0],
                                item_in_box_coor[1],
                                item_in_box_coor[2] + item_in_box_dim[2],
                            )

                        # if gravity check is enabled, generate all coordinate to try
                        if consider_gravity:
                            z_placing_coor_to_try = self.get_all_z_placing_coor(placing_coor)
                        else:
                            z_placing_coor_to_try = [placing_coor]

                        for placing_coor_with_z in z_placing_coor_to_try:

                            # check if exceeded box limit
                            if self.has_exceeded_box_limit(placing_coor_with_z, item_dim):
                                continue
                            fit = True

                            # check for intersection
                            for item_in_box_inter_check in self.items_in_box:
                                inter_check_coor = item_in_box_inter_check.coor
                                inter_check_dim = item_in_box_inter_check.get_rotated_dimension()
                                if self.has_intersection(
                                    inter_check_coor, inter_check_dim, placing_coor_with_z, item_dim
                                ):
                                    fit = False

                            # update coordinate if all check pass, item can fit
                            if fit:
                                self.items_in_box.append(item)
                                return (True, placing_coor_with_z)

            # all options exhausted, return cant fit
            return (False, (0, 0, 0))


class Item:
    def __init__(self, length, width, height, item_id, name="item", colour=None):
        self.length = round(length, NUMBER_OF_DECIMAL)
        self.width = round(width, NUMBER_OF_DECIMAL)
        self.height = round(height, NUMBER_OF_DECIMAL)
        self.item_id = item_id  # unique, ar marker for now
        self.name = name
        self.coor = None
        self.rotation_type = "LWH"
        self.available_rotations = ROTATION
        self.successfully_boxed = False
        if colour:
            self.colour = colour
        else:
            self.colour = [random.random() for _ in range(3)]

    def get_volume(self):
        return self.length * self.width * self.height

    def get_top_surface(self):
        length, width, _ = self.get_rotated_dimension()
        return length * width

    def print_item(self):
        print(
            f"[Item] name: {self.name}, size: {(self.length, self.width, self.height)}, volume: {self.get_volume()}, coor: {self.coor}"
        )

    # set the rotation config
    def set_rotation_type(self, rotation_type):
        self.rotation_type = rotation_type

    # get the dimension, affected by the curent rotation config
    def get_rotated_dimension(self):
        if self.rotation_type == "LWH":
            return (self.length, self.width, self.height)
        elif self.rotation_type == "WLH":
            return (self.width, self.length, self.height)
        elif self.rotation_type == "LHW":
            return (self.length, self.height, self.width)
        elif self.rotation_type == "WHL":
            return (self.width, self.height, self.length)
        elif self.rotation_type == "HWL":
            return (self.height, self.width, self.length)
        elif self.rotation_type == "HLW":
            return (self.height, self.length, self.width)
        else:
            pass

    def reset(self):
        self.rotation_type = "LWH"
        self.available_rotations = ROTATION
        self.successfully_boxed = False
        self.coor = None

    def update_successful_boxed(self, coor):
        self.successfully_boxed = True
        self.coor = coor
        print(f"[Item] item name {self.name} successfully boxed, at coor: {self.coor}")


class Packer:
    def __init__(self):
        # self.box = Box(20, 20, 20, "null")
        self.box = None
        self.item_to_pack = []
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.packing_detail_text_component = None
        self.fig_by_step = None
        self.ax_by_step = None
        self.counter_by_step = -1
        self.received_pack_request = False
        self.get_next_coor_index = 0

    def test_plt(self, _):
        try:
            if self.box == None:
                return
            self.ax.clear()

            # draw the box
            box_x = [0, self.box.length, self.box.length, 0, 0]
            box_y = [0, 0, self.box.width, self.box.width, 0]
            box_z = [0, 0, 0, 0, 0]
            box_x2 = [0, self.box.length, self.box.length, 0, 0]
            box_y2 = [0, 0, self.box.width, self.box.width, 0]
            box_z2 = [self.box.height, self.box.height, self.box.height, self.box.height, self.box.height]
            self.ax.plot(box_x, box_y, box_z, color="b")
            self.ax.plot(box_x2, box_y2, box_z2, color="b")
            for i in range(4):
                self.ax.plot([box_x[i], box_x2[i]], [box_y[i], box_y2[i]], [box_z[i], box_z2[i]], color="b")

            legend_icons = []
            legend_labels = []

            for item in self.box.items_in_box:
                legend_icons.append(
                    plt.Rectangle((0.5, 0.5), width=0.2, height=0.2, edgecolor=item.colour, facecolor=item.colour)
                )
                legend_labels.append(item.name)
                item_x_origin, item_y_origin, item_z_origin = item.coor
                item_length, item_width, item_height = item.get_rotated_dimension()
                self.ax.text(
                    item_x_origin + 0.5 * item_length,
                    item_y_origin + 0.5 * item_width,
                    item_z_origin + 0.5 * item_height,
                    item.name,
                    color="black",
                    fontsize=12,
                )
                item_x = [
                    item_x_origin,
                    item_x_origin + item_length,
                    item_x_origin + item_length,
                    item_x_origin,
                    item_x_origin,
                ]
                item_y = [
                    item_y_origin,
                    item_y_origin,
                    item_y_origin + item_width,
                    item_y_origin + item_width,
                    item_y_origin,
                ]
                item_z = [item_z_origin, item_z_origin, item_z_origin, item_z_origin, item_z_origin]

                item_x2 = [x for x in item_x]
                item_y2 = [y for y in item_y]
                item_z2 = [
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                ]
                for i in range(4):
                    self.ax.plot_surface(
                        np.array([item_x[i : i + 2], item_x2[i : i + 2]]),
                        np.array([item_y[i : i + 2], item_y2[i : i + 2]]),
                        np.array([item_z[i : i + 2], item_z2[i : i + 2]]),
                        color=item.colour,
                        alpha=0.4,
                    )
                self.ax.plot_surface(
                    np.array([item_x, item_x2, item_x2[::-1], item_x[::-1], item_x]),
                    np.array([item_y, item_y, item_y, item_y, item_y]),
                    np.array([item_z, item_z2, item_z2, item_z, item_z]),
                    color=item.colour,
                    alpha=0.4,
                )
            # draw the axes, title, text etc
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")
            self.ax.set_title(self.box.name)
            packing_detail_text = f"Successfully packed {len(self.box.items_in_box)} / {len(self.item_to_pack)} items"
            if self.packing_detail_text_component == None:
                self.packing_detail_text_component = self.fig.text(
                    0.5,
                    0.01,
                    packing_detail_text,
                    ha="center",
                    va="bottom",
                    fontsize=12,
                )
            else:
                self.packing_detail_text_component.set_text(packing_detail_text)
            if len(legend_labels):
                plt.legend(handles=legend_icons, labels=legend_labels, loc="upper right")
            return self.ax
        except:
            print("[Visualizer] Error then plotting 3D plot, please check")
            return

    # most outer layer of packing function, called by ui
    def pack(self, consider_stability=True, consider_gravity=True, enable_3d_rotation=True):
        global ROTATION
        print(
            f"called pack consider_stability: {consider_stability}, consider_gravity:{consider_gravity}, enable_3d_rotation: {enable_3d_rotation}"
        )
        if enable_3d_rotation:
            ROTATION = ["LWH", "WLH", "LHW", "WHL", "HWL", "HLW"]
        else:
            ROTATION = ["LWH", "WLH"]

        # reset the box and items
        self.box.reset()
        for item in self.item_to_pack:
            item.reset()
            item.print_item()

        if consider_stability:
            # if stability check and full rotation is enabled, check surface area for all rotation config and sort in decending order,
            # also set the item to be in the largest surface area rotation config
            if enable_3d_rotation:
                for item in self.item_to_pack:
                    surface_area_rot_dict = {}
                    for rotation_type in ROTATION:
                        item.set_rotation_type(rotation_type)
                        surface_area = item.get_top_surface()
                        surface_area_rot_dict[rotation_type] = surface_area
                    print(surface_area_rot_dict)
                    item.available_rotations = sorted(
                        surface_area_rot_dict, key=lambda k: surface_area_rot_dict[k], reverse=True
                    )
                    item.set_rotation_type(item.available_rotations[0])
            # determine the packing order, based on surface area, biggest first
            self.item_to_pack.sort(key=lambda item: item.get_top_surface(), reverse=True)

        # determine the packing order, based on volume, biggest first
        else:
            self.item_to_pack.sort(key=lambda item: item.get_volume(), reverse=True)
        print(f"pius debug {self.item_to_pack}")
        for item in self.item_to_pack:
            print(f"pius debug, processing {item}")

            # call to pack item in box, if packing is successful, update the item's coordinate
            success, coor = self.box.put_item(item, consider_gravity)
            if success:
                item.update_successful_boxed(coor)

        # post packing
        print(f"[Packer] Successfully packed {len(self.box.items_in_box)} / {len(self.item_to_pack)} items")
        # sort the packed item in bottom-up manner, useful for robot integration later on
        self.box.items_in_box = sorted(
            self.box.items_in_box, key=lambda item: (item.coor[2], item.coor[1], item.coor[0])
        )
        for item in self.box.items_in_box:
            print(
                f"[Packer] {item.name} is at {item.coor} with rotation config: {item.rotation_type}, rotated dim: {item.get_rotated_dimension()}"
            )

    def visualization(self):
        try:
            if self.fig is None or self.ax is None:
                self.fig = plt.figure()
                self.ax = self.fig.add_subplot(111, projection="3d")
            self.ax.clear()
            # draw the box
            box_x = [0, self.box.length, self.box.length, 0, 0]
            box_y = [0, 0, self.box.width, self.box.width, 0]
            box_z = [0, 0, 0, 0, 0]
            box_x2 = [0, self.box.length, self.box.length, 0, 0]
            box_y2 = [0, 0, self.box.width, self.box.width, 0]
            box_z2 = [self.box.height, self.box.height, self.box.height, self.box.height, self.box.height]
            self.ax.plot(box_x, box_y, box_z, color="b")
            self.ax.plot(box_x2, box_y2, box_z2, color="b")
            for i in range(4):
                self.ax.plot([box_x[i], box_x2[i]], [box_y[i], box_y2[i]], [box_z[i], box_z2[i]], color="b")

            legend_icons = []
            legend_labels = []

            # draw packed item
            for item in self.box.items_in_box:
                legend_icons.append(
                    plt.Rectangle((0.5, 0.5), width=0.2, height=0.2, edgecolor=item.colour, facecolor=item.colour)
                )
                legend_labels.append(item.name)
                item_x_origin, item_y_origin, item_z_origin = item.coor
                item_length, item_width, item_height = item.get_rotated_dimension()
                self.ax.text(
                    item_x_origin + 0.5 * item_length,
                    item_y_origin + 0.5 * item_width,
                    item_z_origin + 0.5 * item_height,
                    item.name,
                    color="black",
                    fontsize=12,
                )
                item_x = [
                    item_x_origin,
                    item_x_origin + item_length,
                    item_x_origin + item_length,
                    item_x_origin,
                    item_x_origin,
                ]
                item_y = [
                    item_y_origin,
                    item_y_origin,
                    item_y_origin + item_width,
                    item_y_origin + item_width,
                    item_y_origin,
                ]
                item_z = [item_z_origin, item_z_origin, item_z_origin, item_z_origin, item_z_origin]

                item_x2 = [x for x in item_x]
                item_y2 = [y for y in item_y]
                item_z2 = [
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                    item_z_origin + item_height,
                ]
                for i in range(4):
                    self.ax.plot_surface(
                        np.array([item_x[i : i + 2], item_x2[i : i + 2]]),
                        np.array([item_y[i : i + 2], item_y2[i : i + 2]]),
                        np.array([item_z[i : i + 2], item_z2[i : i + 2]]),
                        color=item.colour,
                        alpha=0.4,
                    )
                self.ax.plot_surface(
                    np.array([item_x, item_x2, item_x2[::-1], item_x[::-1], item_x]),
                    np.array([item_y, item_y, item_y, item_y, item_y]),
                    np.array([item_z, item_z2, item_z2, item_z, item_z]),
                    color=item.colour,
                    alpha=0.4,
                )

            # draw the axes, title, text etc
            self.ax.set_xlabel("X")
            self.ax.set_ylabel("Y")
            self.ax.set_zlabel("Z")
            self.ax.set_title(self.box.name)
            packing_detail_text = f"Successfully packed {len(self.box.items_in_box)} / {len(self.item_to_pack)} items"
            if self.packing_detail_text_component == None:
                self.packing_detail_text_component = self.fig.text(
                    0.5,
                    0.01,
                    packing_detail_text,
                    ha="center",
                    va="bottom",
                    fontsize=12,
                )
            else:
                self.packing_detail_text_component.set_text(packing_detail_text)
            plt.legend(handles=legend_icons, labels=legend_labels, loc="upper right")
            plt.draw()
        except:
            pass


class BinPacker(Node):
    def __init__(self):
        super().__init__("bin_packer")
        self.packer = Packer()
        self.create_service(SetBox, "set_box", self.set_box_cb)
        self.create_service(SetItem, "set_item", self.set_item_cb)
        self.create_service(Trigger, "pack", self.request_to_pack_cb)
        self.create_service(GetNextPose, "get_next_placing_pose", self.get_next_coor_cb)
        self.create_service(Trigger, "reset", self.reset_cb)
        # self.srv = self.create_service(StringTrigger, "delete_item", self.delete_item_cb)

    def set_box_cb(self, req, res):
        self.get_logger().info(f"Set Box service called")
        try:
            if self.packer.box != None:
                self.get_logger().warn("Box has been set previously, will overwrite with new Box config")
                self.packer.received_pack_request = False
            self.packer.box = Box(req.box.length, req.box.width, req.box.height, req.pose, req.box.name)
            res.success = True
            return res

        except Exception as e:
            self.get_logger().warn(f"Set Box service has encountered a error: {e}")
            res.success = False
            return res

    def set_item_cb(self, req, res):
        self.get_logger().info(f"Set Item service called")
        try:
            if self.packer.box == None:
                self.get_logger().error("No Box is set, please set a box before proceeding")
                res.success = False
                return res
            self.packer.received_pack_request = False
            self.packer.item_to_pack.append(
                Item(
                    req.item.length,
                    req.item.width,
                    req.item.height,
                    req.item_id,
                    req.item.name,
                    req.item.color,
                )
            )
            res.success = True
            return res

        except Exception as e:
            self.get_logger().warn(f"Set Item service has encountered a error: {e}")
            res.success = False
            return res

    def request_to_pack_cb(self, req, res):
        try:
            self.packer.pack()
            self.packer.received_pack_request = True
            self.packer.get_next_coor_index = 0
            res.success = True
            return res
        except Exception as e:
            self.get_logger().warn(f"Pack request service has encountered a error: {e}")
            res.success = False
            return res

    def get_next_coor_cb(self, req, res):
        print("received req to send next coor")
        if not self.packer.received_pack_request:
            self.get_logger().warn(f"Please call the pack service first before continuing")
            res.output = GetNextPose.Response.ERROR
            return res
        if (self.packer.get_next_coor_index + 1) > len(self.packer.box.items_in_box):
            res.output = GetNextPose.Response.COMPLETED
            return res
        item = self.packer.box.items_in_box[self.packer.get_next_coor_index]
        print(f'return item {item.item_id}')
        if not item.successfully_boxed:
            self.get_logger().warn(f"Item is not boxed, something is wrong, please retry the packing")
            res.success = False
            return res
        res.item_id = item.item_id
        res.rotation_config = item.rotation_type
        box_pose = self.packer.box.pose
        (x, y, z) = item.coor
        (dim_x, dim_y, dim_z) = item.get_rotated_dimension()

        res.placing_pose = Pose()
        res.placing_pose.position.x = box_pose.position.x + x + 0.5 * dim_x
        res.placing_pose.position.y = box_pose.position.y + y + 0.5 * dim_y
        res.placing_pose.position.z = box_pose.position.z + z + dim_z
        res.placing_pose.orientation = box_pose.orientation
        res.output = GetNextPose.Response.SUCCESS

        self.packer.get_next_coor_index += 1
        return res

    def reset_cb(self, req, res):
        print("Request to reset the bin packer")
        self.packer.box = None
        self.packer.item_to_pack = []
        self.packer.packing_detail_text_component = None
        self.packer.fig_by_step = None
        self.packer.ax_by_step = None
        self.packer.counter_by_step = -1
        self.packer.received_pack_request = False
        self.packer.get_next_coor_index = 0
        res.success = True
        return res

    def start_plt(self):
        self.ani = anim.FuncAnimation(self.packer.fig, self.packer.test_plt, interval=1000)
        plt.show()


def main(args=None):
    rclpy.init(args=args)
    bin_packer = BinPacker()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(bin_packer)
    executor.spin()
    # thread = threading.Thread(target=executor.spin, daemon=True)
    # thread.start()
    # bin_packer.start_plt()


if __name__ == "__main__":
    main()
