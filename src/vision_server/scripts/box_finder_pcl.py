#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import pcl
import numpy as np
from std_msgs.msg import Header
import tf2_ros
from geometry_msgs.msg import TransformStamped
from bin_packing_msgs.srv import *
from std_srvs.srv import Trigger
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

class BoxDimensionEstimator(Node):
    def __init__(self):
        super().__init__("box_dimension_estimator")
        self.create_service(Trigger, "start", self.get_all_box_detail_callback)
        # self.get_all_box_detail_srv = self.create_client(
        #     GetAllBoxDetail, "get_all_box_detail", callback_group=MutuallyExclusiveCallbackGroup()
        # )
        self.subscription = self.create_subscription(
            PointCloud2, "/camera/camera/depth/color/points", self.listener_callback, 10  # Topic name for RealSense
        )
        self.publisher_ = self.create_publisher(PointCloud2, "/downsampled/points", 10)  # Output topic
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.subscription
    def get_all_box_detail_callback(self, request, response):
        self.get_logger().info(f"Received request to get all box details")
        req = GetAllBoxDetail.Request()
        get_all_box_detail_res = self.get_all_box_detail_srv.call(req)
        if get_all_box_detail_res.success:
            boxes = get_all_box_detail_res.boxes
            for index, box in enumerate(boxes):
                
                self.get_logger().info(f"Box ID: {index}, length: {box.shape.length}, width: {box.shape.width}, height: {box.shape.height}")
        response.success = True
        return response
        
    def listener_callback(self, msg):
        # Convert ROS2 PointCloud2 to a NumPy array
        # cloud_points = np.array([p for p in pc2.read_points(msg, skip_nans=True)])
        cloud_points = np.array(
            [(p[0], p[1], p[2]) for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)]
        )

        # Create a PCL point cloud from NumPy array
        cloud = pcl.PointCloud()
        cloud.from_array(cloud_points.astype(np.float32))
        
        # Apply Z-axis filtering using PassThrough filter
        passthrough = cloud.make_passthrough_filter()
        passthrough.set_filter_field_name("z")
        passthrough.set_filter_limits(0.0, 0.4)  # Keep points with Z <= 70 cm (0.7 meters)
        filtered_cloud = passthrough.filter()
        
        # Apply voxel grid downsampling
        sor = filtered_cloud.make_voxel_grid_filter()
        leaf_size = 0.01  # Adjust this value for your desired downsample size
        sor.set_leaf_size(leaf_size, leaf_size, leaf_size)
        downsampled_cloud = sor.filter()

        # Step 3: Segment the tabletop plane using RANSAC
        seg = downsampled_cloud.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_PLANE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold(0.01)  # Distance threshold to be considered part of the plane
        inliers, coefficients = seg.segment()
        objects_cloud = downsampled_cloud.extract(inliers, negative=True)
        
        sor = objects_cloud.make_statistical_outlier_filter()
        sor.set_mean_k(50)
        sor.set_std_dev_mul_thresh(1.0)
        filtered_objects_cloud = sor.filter()
        
        # Step 4: Perform Euclidean Cluster Extraction to separate individual objects
        tree = filtered_objects_cloud.make_kdtree()
        ec = filtered_objects_cloud.make_EuclideanClusterExtraction()
        ec.set_ClusterTolerance(0.02)  # Set tolerance based on your object size (in meters)
        ec.set_MinClusterSize(150)
        ec.set_MaxClusterSize(25000)
        ec.set_SearchMethod(tree)
        cluster_indices = ec.Extract()
        
        for j, indices in enumerate(cluster_indices):
            points = np.asarray([filtered_objects_cloud[i] for i in indices])

            # Perform PCA to get the principal axes
            mean = np.mean(points, axis=0)  # This gives you the center in 3D (x, y, z)
            centered_points = points - mean
            cov_matrix = np.cov(centered_points, rowvar=False)
            eigenvalues, eigenvectors = np.linalg.eigh(cov_matrix)

            # Sort eigenvectors by eigenvalues (largest to smallest)
            sort_indices = np.argsort(eigenvalues)[::-1]
            eigenvectors = eigenvectors[:, sort_indices]

            # The first two principal components give the orientation in the xy-plane
            principal_axis = eigenvectors[:, 0]  # Principal axis
            yaw = np.arctan2(principal_axis[1], principal_axis[0])  # Yaw angle in radians

            # Calculate the OBB by projecting points onto the principal axes
            projected_points = np.dot(centered_points, eigenvectors)
            min_point = np.min(projected_points, axis=0)
            max_point = np.max(projected_points, axis=0)
            obb_size = max_point - min_point

            # Center of the box (adjusted for the OBB orientation)
            obb_center = mean  # The mean is already the center of the box

            # Adjust Z for the top surface of the box
            obb_center[2] = np.max(points[:, 2])  # Set z-coordinate to the top surface

            # Convert yaw to degrees
            yaw_degrees = np.degrees(yaw)

            # Log the size, center, and yaw of the box
            self.get_logger().info(f"Box {j+1}: size (x, y, z) = {obb_size}, yaw = {yaw_degrees} degrees, center (x, y, z) = {obb_center}")
            
            t = TransformStamped()
            t.header.stamp = self.get_clock().now().to_msg()
            t.header.frame_id = "camera_depth_optical_frame"  # Replace with your parent frame
            t.child_frame_id = f"Box {j+1}"  # Replace with your child frame

            # Set translation
            t.transform.translation.x = obb_center[0]
            t.transform.translation.y = obb_center[1]
            t.transform.translation.z = obb_center[2]

            # Set rotation (quaternion)
            t.transform.rotation.w = 1.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = 0.0

            # Broadcast the transform
            self.tf_broadcaster.sendTransform(t)
        
        # Convert the downsampled point cloud back to ROS2 PointCloud2 message
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = msg.header.frame_id  # Preserve the original frame ID
        downsampled_msg = pc2.create_cloud_xyz32(header, np.asarray(filtered_cloud))

        # Publish the downsampled point cloud
        self.publisher_.publish(downsampled_msg)

        # Optional: log the size of the downsampled cloud for debugging
        self.get_logger().info(f"Published downsampled point cloud with {downsampled_cloud.size} points")


def main(args=None):
    rclpy.init(args=args)
    master_controller = BoxDimensionEstimator()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(master_controller)
    executor.spin()
    executor.shutdown()
    master_controller.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
