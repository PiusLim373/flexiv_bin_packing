#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from geometry_msgs.msg import TransformStamped
import numpy as np
import tf2_ros
from icecream import ic
import transforms3d

ic.configureOutput(includeContext=True)
np.set_printoptions(suppress=True)


class CameraTFHandler(Node):

    def __init__(self):
        super().__init__("camera_tf_handler")
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        transform = self.tf_buffer.lookup_transform("camera_color_optical_frame", "camera_link", rclpy.time.Time())
        optical_T_cam = self.transform_to_t_mat(transform)

        flange_T_optical = np.array(
            [
                [-0.0165495, 0.999792, -0.0119133, -0.0531654],
                [-0.999807, -0.0164208, 0.0108253, 0.0265602],
                [0.0106274, 0.0120901, 0.99987, 0.0168167],
                [0, 0, 0, 1],
            ]
            # fist successful calibration
            # [
            #     [-0.000147777, 0.999937, -0.0112616, -0.0534386],
            #     [-0.99987, -0.000329098, -0.0161006, 0.0435051],
            #     [-0.0161033, 0.0112578, 0.999807, 0.0191439],
            #     [0, 0, 0, 1],
            # ]
        )
        flange_T_cam = np.dot(flange_T_optical, optical_T_cam)
        self.publish_transform(flange_T_cam)

    def transform_to_t_mat(self, transform):
        translation = transform.transform.translation
        translation_vector = np.array([translation.x, translation.y, translation.z])

        # Extract rotation (quaternion)
        rotation = transform.transform.rotation
        quaternion = [rotation.w, rotation.x, rotation.y, rotation.z]

        # Convert quaternion to a 3x3 rotation matrix
        rotation_matrix = transforms3d.quaternions.quat2mat(quaternion)

        # Form the 4x4 transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = rotation_matrix
        transformation_matrix[:3, 3] = translation_vector
        return transformation_matrix

    def publish_transform(self, transformation_matrix):
        translation = transformation_matrix[:3, 3]

        # Extract rotation matrix (top-left 3x3 part of the 4x4 matrix)
        rotation_matrix = transformation_matrix[:3, :3]

        # Convert rotation matrix to quaternion
        quaternion = transforms3d.quaternions.mat2quat(rotation_matrix)  # Returns [w, x, y, z]

        # Create the TransformStamped message
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "flange"  # Replace with your parent frame
        t.child_frame_id = "camera_link"  # Replace with your child frame

        # Set translation
        t.transform.translation.x = translation[0]
        t.transform.translation.y = translation[1]
        t.transform.translation.z = translation[2]

        # Set rotation (quaternion)
        t.transform.rotation.w = quaternion[0]
        t.transform.rotation.x = quaternion[1]
        t.transform.rotation.y = quaternion[2]
        t.transform.rotation.z = quaternion[3]

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    camera_tf_handler = CameraTFHandler()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(camera_tf_handler)
    executor.spin()
    executor.shutdown()
    camera_tf_handler.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
