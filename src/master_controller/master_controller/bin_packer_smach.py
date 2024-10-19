#!/usr/bin/python3

import rclpy
import rclpy.subscription
from rclpy.node import Node
import smach
from smach_ros import IntrospectionServer
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from std_srvs.srv import *
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time

class GetAllItemAndPickSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["success"])
        self.counter = 0
        self.node = node
        self.srv = self.node.create_client(
            Trigger, "get_all_item_and_pack", callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute(self, userdata):
        self.node.get_logger().info("Executing state GetAllItemAndPickSmach")
        future = self.srv.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if not res.success:
            self.node.get_logger().error("Error in GetAllItemAndPickSmach")
        return "success"


class GetNextCoordinateSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["error_finding_pose", "transfer_item", "completed"])
        self.node = node
        self.srv = self.node.create_client(
            GetNextCoordinate, "get_next_coordinate", callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute(self, userdata):
        self.node.get_logger().info("Executing state GetNextCoordinateSmach")
        req = GetNextCoordinate.Request()
        future = self.srv.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if res.outcome == GetNextCoordinate.Response.SUCCESS:
            return "transfer_item"
        elif res.outcome == GetNextCoordinate.Response.ERROR:
            return "error_finding_pose"
        elif res.outcome == GetNextCoordinate.Response.COMPLETED:
            return "completed"


class ResetBinPackerSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["success"])
        self.node = node
        self.srv = self.node.create_client(Trigger, "reset_bin_packer", callback_group=MutuallyExclusiveCallbackGroup())

    def execute(self, userdata):
        self.node.get_logger().info("Executing state ResetBinPackerSmach")
        future = self.srv.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if not res.success:
            self.node.get_logger().error("Error in ResetBinPackerSmach")
        return "success"


class TransferItemSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["success", "transferred_to_fm", "aborted"])
        self.node = node
        self.srv = self.node.create_client(
            TransferItem, "transfer_item", callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute(self, userdata):
        self.node.get_logger().info("Executing state TransferItemSmach")
        future = self.srv.call_async(TransferItem.Request())
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if res.outcome == TransferItem.Response.BOX:
            return "success"
        elif res.outcome == TransferItem.Response.FLIPPING_MECHANISM:
            return "transferred_to_fm"
        else:
            self.node.get_logger().error("Error in TransferItemSmach")
            return "aborted"


class TransferFromFMSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["success", "aborted"])
        self.node = node
        self.srv = self.node.create_client(
            Trigger, "transfer_from_flipping_mechanism", callback_group=MutuallyExclusiveCallbackGroup()
        )

    def execute(self, userdata):
        self.node.get_logger().info("Executing state TransferFromFMSmach")
        future = self.srv.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self.node, future)
        res = future.result()
        if res.success:
            return "success"
        else:
            self.node.get_logger().error("Error in TransferFromFMSmach")
            return "aborted"
        
class WaitingForStartSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["success"])
        self.node = node
        

    def execute(self, userdata):
        input("press enter to start")
        return "success"
    
class BarSmach(smach.State):
    def __init__(self, node: Node):
        smach.State.__init__(self, outcomes=["success"])
        self.node = node
        

    def execute(self, userdata):
        time.sleep(1)
        return "success"
    
# main function
def main():
    rclpy.init()

    # Create a node
    node = rclpy.create_node("smach_example_state_machine")

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=["completed", "aborted"])
    sis = IntrospectionServer('state_machine_introspection', sm, '/SM_ROOT2')
    sis.start()
    
    # Open the container
    with sm:
        # smach.StateMachine.add(
        #     "FooSmach", FooSmach(node), transitions={"success": "BarSmach"}
        # )
        smach.StateMachine.add(
            "WaitingForStartSmach", WaitingForStartSmach(node), transitions={"success": "GetAllItemAndPickSmach"}
        )
        smach.StateMachine.add(
            "GetAllItemAndPickSmach", GetAllItemAndPickSmach(node), transitions={"success": "GetNextCoordinateSmach"}
        )
        smach.StateMachine.add(
            "GetNextCoordinateSmach",
            GetNextCoordinateSmach(node),
            transitions={
                "transfer_item": "TransferItemSmach",
                "error_finding_pose": "ResetBinPackerSmach",
                "completed": "completed",
            },
        )
        smach.StateMachine.add(
            "ResetBinPackerSmach", ResetBinPackerSmach(node), transitions={"success": "GetAllItemAndPickSmach"}
        )
        smach.StateMachine.add(
            "TransferItemSmach",
            TransferItemSmach(node),
            transitions={
                "success": "GetNextCoordinateSmach",
                "transferred_to_fm": "TransferFromFMSmach",
                "aborted": "aborted",
            },
        )
        smach.StateMachine.add(
            "TransferFromFMSmach",
            TransferFromFMSmach(node),
            transitions={"success": "GetNextCoordinateSmach", "aborted": "aborted"},
        )

    # Execute SMACH plan
    outcome = sm.execute()

    # Shutdown node
    sis.stop()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
