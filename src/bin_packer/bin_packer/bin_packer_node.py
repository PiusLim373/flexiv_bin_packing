#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from std_srvs.srv import *

import random

class Box:
    def __init__(self, length, width, height, name="box"):
        self.length = length
        self.width = width
        self.height = height
        self.name = name
        self.items_in_box = []


class Item:
    def __init__(self, length, width, height, name="item", colour=None):
        self.length = length
        self.width = width
        self.height = height

class Item:
    def __init__(self, length, width, height, name="item", colour=None):
        self.length = length
        self.width = width
        self.height = height
        self.name = name
        self.coor = None
        self.rotation_type = "LWH"
        self.available_rotations = ROTATION
        self.successfully_boxed = False
        if colour:
            self.colour = colour
        else:
            self.colour = [random.random() for _ in range(3)]


class Packer:
    def __init__(self):
        self.box = None
        self.item_to_pack = []
        self.fig = None
        self.ax = None
        self.packing_detail_text_component = None
        self.fig_by_step = None
        self.ax_by_step = None
        self.counter_by_step = -1
        self.recived_pack_request = False


class BinPacker(Node):
    def __init__(self):
        self.srv = self.create_service(SetBox, "set_box", self.set_box_cb)
        self.srv = self.create_service(SetItem, "set_item", self.set_item_cb)
        self.srv = self.create_service(GetNextPose, "get_next_placing_coor", self.get_next_coor_cb)
        self.srv = self.create_service(Trigger, "pack", self.request_to_pack_cb)
        self.srv = self.create_service(StringTrigger, "delete_item", self.delete_item_cb)
        self.packer = Packer()
    
    def set_box_cb(self, req, res):
        try:
            if self.packer.box != None:
                self.get_logger().warn("Box has been set previously, will overwrite with new Box config")
                self.packer.recived_pack_request = False

        except Exception as e:
            self.get_logger().warn(f"Set Box service has encountered a error: {e}")
            res.success = False
            return res



def main(args=None):
    rclpy.init(args=args)
    bin_packer = BinPacker()
    rclpy.spin(bin_packer)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
