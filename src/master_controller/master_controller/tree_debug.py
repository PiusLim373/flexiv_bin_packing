#!/usr/bin/python3
import bin_packing_msgs.action
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
import bin_packing_msgs
import random
import time
from std_srvs.srv import Trigger, SetBool
from rclpy.action import ActionServer
import asyncio
class MasterController(Node):

    def __init__(self):
        super().__init__("master_controller")
        
        self._action_server = ActionServer(
            self,
            bin_packing_msgs.action.Trigger,
            'get_all_item_and_pack_action',
            self.get_all_item_and_pack_action_cb
        )
        
        self.create_service(
            Trigger, "get_all_item_and_pack", self.get_all_item_and_pack_cb
        )  # a service that populate data for item to be transder
        # services
        self.create_service(
            GetNextCoordinate, "get_next_coordinate", self.get_next_coordinate_cb
        )  # a service that populate data for item to be transder
        self.create_service(
            TransferItem, "transfer_item", self.transfer_item_cb
        )  # a service that pick the item based on rotational configuration set
        self.create_service(
            Trigger, "transfer_from_flipping_mechanism", self.transfer_from_flipping_mechanism_cb
        )  # a service that transfer item from fm based on rotational configuration set
        self.create_service(
            Trigger, "reset_bin_packer", self.reset_bin_packer_cb
        )  # a service that transfer item from fm based on rotational configuration set
        
        self.counter = 0
        print("tree debug ready")

    def get_all_item_and_pack_action_cb(self, goal_handle):
        self.get_logger().info("Getting all boxes and packing")
        feedback_msg = bin_packing_msgs.action.Trigger.Feedback()
        start_time = time.time()
        while time.time() - start_time < 5 :
            print("Working")
            feedback_msg.feedback = "i am working"
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)
        result = bin_packing_msgs.action.Trigger.Result()
        result.success = True
        goal_handle.succeed()
        return result
    
    def get_all_item_and_pack_cb(self, request, response):
        self.get_logger().info("get_all_item_and_pack_cb")
        response.success = True
        delay = random.randint(1, 4)
        delay = 5
        print("Delaying for", delay, "seconds")
        # time.sleep(delay)
        print("Done delaying")
        return response
    
    def get_next_coordinate_cb(self, request, response):
        self.get_logger().info("get_next_coordinate_cb")
        # random select from [0,1,2]
        random_choice = random.choice([0, 1, 2])
        if self.counter == 5:
            random_choice = 2
        self.get_logger().info(f"Random choice: {random_choice}")
        if random_choice == 0:
            response.outcome = GetNextCoordinate.Response.ERROR
        elif random_choice == 1:
            response.outcome = GetNextCoordinate.Response.SUCCESS
        elif random_choice == 2:
            response.outcome = GetNextCoordinate.Response.COMPLETED
            self.counter = 0
        delay = random.randint(2, 5)
        print("Delaying for", delay, "seconds")
        # time.sleep(delay)
        print("Done delaying")
        self.counter += 1
        return response
    
    def reset_bin_packer_cb(self, request, response):
        self.get_logger().info("reset_bin_packer_cb")
        response.success = True
        return response 
    
    def transfer_item_cb(self, request, response):
        self.get_logger().info("transfer_item_cb")
        response.outcome = random.choice([1, 2])
        delay = random.randint(1, 4)
        print("Delaying for", delay, "seconds")
        # time.sleep(delay)
        print("Done delaying")
        return response

    def transfer_from_flipping_mechanism_cb(self, request, response):
        self.get_logger().info("transfer_from_flipping_mechanism_cb")
        response.success = True
        delay = random.randint(1, 4)
        print("Delaying for", delay, "seconds")
        # time.sleep(delay)
        print("Done delaying")
        return response


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
