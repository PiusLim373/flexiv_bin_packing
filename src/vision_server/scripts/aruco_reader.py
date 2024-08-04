#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import tf2_ros
import pyrealsense2 as rs
from std_srvs.srv import Trigger


class ArcuoReader(Node):

    def __init__(self):
        super().__init__("aruco_reader")
        self.create_service(GetAllAruco, "get_all_aruco", self.get_all_aruco_cb)
        self.create_service(GetArucoPose, "get_aruco_pose", self.get_aruco_pose_cb)
        self.image_publisher = self.create_publisher(Image, "aruco_reader_output", 10)
        self.bridge = CvBridge()
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.image_cb, 10)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.depth_image_cb, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.camera_info_cb, 10)
        self.depth_camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.depth_camera_info_cb, 10
        )
        self.image = None
        self.depth_image = None
        self.pointcloud = None
        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
        self.parameters = cv2.aruco.DetectorParameters_create()
        self.marker_length = 0.05
        self.camera_matrix = None
        self.dist_coeffs = None
        self.intrinsics = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_service(GetArucoPose, "debug", self.debug_cb)
        self.get_pixel_point_srv = self.create_client(
            GetPixelPoint, "get_pixel_point", callback_group=MutuallyExclusiveCallbackGroup()
        )

    def debug_cb(self, req, res):
        self.get_logger().info("Got a request to get aruco point")
        corners, ids, _ = cv2.aruco.detectMarkers(self.image, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            all_id_list = ids.flatten().tolist()
            if req.marker_id in all_id_list:
                marker_index = all_id_list.index(req.marker_id)
                marker_corners = corners[marker_index][0]
                center = marker_corners.mean(axis=0).astype(int)
                cv_image = cv2.aruco.drawDetectedMarkers(self.image, corners, ids)
                cv2.circle(cv_image, tuple(center), 5, (0, 0, 255), -1)
                u = int(marker_corners[:, 0].mean())
                v = int(marker_corners[:, 1].mean())
                self.get_logger().info(f"Marker ID {req.marker_id} found at ({u}, {v})")
                get_pixel_point_req = GetPixelPoint.Request()
                get_pixel_point_req.u = u
                get_pixel_point_req.v = v
                get_pixel_point_res = self.get_pixel_point_srv.call(get_pixel_point_req)
                print(get_pixel_point_res)
                if get_pixel_point_res.success:

                    if self.camera_matrix is None or self.dist_coeffs is None:
                        self.get_logger().warn("Camera info not yet received, skipping aruco orientation for now")
                        res.success = False
                        return res
                    else:
                        rvecs, _, _ = cv2.aruco.estimatePoseSingleMarkers(
                            corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                        )
                        rotation_matrix, _ = cv2.Rodrigues(rvecs[marker_index])
                        quad = self.rotation_matrix_to_quaternion(rotation_matrix)
                        trans = [get_pixel_point_res.point.x, get_pixel_point_res.point.y, get_pixel_point_res.point.z]
                        self.publish_transform(trans, quad, str(ids[marker_index]))
                        res.pose.position = get_pixel_point_res.point
                        res.pose.orientation.x = quad[0]
                        res.pose.orientation.y = quad[1]
                        res.pose.orientation.z = quad[2]
                        res.pose.orientation.w = quad[3]
                        res.success = True
                        return res
                else:
                    self.get_logger().warn(f"Couldn't get point for marker ID: {req.marker_id}")
                    res.success = False
                    return res
        else:
            self.get_logger().warn(f"Couldn't get point for marker ID: {req.marker_id}")
            res.success = False
            return res

    def camera_info_cb(self, msg):
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)
        self.get_logger().info("Camera info received and subscription destroyed.")
        self.destroy_subscription(self.camera_info_sub)

    def depth_camera_info_cb(self, cameraInfo):
        self.intrinsics = rs.intrinsics()
        self.intrinsics.width = cameraInfo.width
        self.intrinsics.height = cameraInfo.height
        self.intrinsics.ppx = cameraInfo.k[2]
        self.intrinsics.ppy = cameraInfo.k[5]
        self.intrinsics.fx = cameraInfo.k[0]
        self.intrinsics.fy = cameraInfo.k[4]
        if cameraInfo.distortion_model == "plumb_bob":
            self.intrinsics.model = rs.distortion.brown_conrady
        elif cameraInfo.distortion_model == "equidistant":
            self.intrinsics.model = rs.distortion.kannala_brandt4
        self.intrinsics.coeffs = [i for i in cameraInfo.d]
        print(self.intrinsics)
        self.get_logger().info("Camera info received and subscription destroyed.")
        self.destroy_subscription(self.depth_camera_info_sub)

    def depth_image_cb(self, msg):
        self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg.encoding)

    def image_cb(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        # cv2.imshow("camera2", self.image)
        # cv2.waitKey(1)
        # corners, ids, _ = cv2.aruco.detectMarkers(self.image, self.aruco_dict, parameters=self.parameters)
        # if ids is not None:
        #         all_id_list = ids.flatten().tolist()
        #         if 0 in all_id_list:
        #             marker_index = all_id_list.index(0)
        #             marker_corners = corners[marker_index][0]
        #             center = marker_corners.mean(axis=0).astype(int)
        #             print(center)

        # cv2.imshow("camera", self.image)
        # cv2.waitKey(1)
        # corners, ids, _ = cv2.aruco.detectMarkers(self.image, self.aruco_dict, parameters=self.parameters)
        # if ids is not None:
        #     cv_image = cv2.aruco.drawDetectedMarkers(self.image, corners, ids)
        #     for i, corner in enumerate(corners):
        #         center = corner[0].mean(axis=0).astype(int)
        #         cv2.circle(cv_image, tuple(center), 5, (0, 0, 255), -1)
                # if self.camera_matrix is None or self.dist_coeffs is None:
                #     self.get_logger().info("Camera info not yet received, skipping aruco orientation for now")
                # else:
                #     rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                #         corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                #     )
                # cv2.aruco.drawAxis(cv_image, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.05)
                # rotation_matrix, _ = cv2.Rodrigues(rvecs[i])
                # self.publish_transform(tvecs[i][0], rotation_matrix, ids[i])

                # rpy = self.rotation_matrix_to_rpy(rotation_matrix)
                # print(rpy)
                # aruco_reader_output = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                # aruco_reader_output.header.frame_id = "camera_color_optical_frame"
                # aruco_reader_output.header.stamp = self.get_clock().now().to_msg()
                # self.image_publisher.publish(aruco_reader_output)
        # cv2.imshow("camera", cv_image)
        # cv2.waitKey(1)

    def get_all_aruco_cb(self, req, res):
        # read the camera image and find all aruco markers
        self.get_logger().info("Got a request to get all aruco markers")
        try:
            _, ids, _ = cv2.aruco.detectMarkers(self.image, self.aruco_dict, parameters=self.parameters)
            if ids is not None:
                self.get_logger().info(f"Markers detected: {ids.flatten().tolist()}")
                res.aruco_markers = ids.flatten().tolist()
                res.success = True
                return res
            else:
                self.get_logger().warn("No marker is detected")
                res.success = False
                return res
        except Exception as a:
            self.get_logger().error(a)
            res.success = False
            return res

    def get_aruco_pose_cb(self, req, res):
        # read the camera image, find the point of the specified marker id
        self.get_logger().info("Got a request to get aruco point")
        try:
            corners, ids, _ = cv2.aruco.detectMarkers(self.image, self.aruco_dict, parameters=self.parameters)
            if ids is not None:
                all_id_list = ids.flatten().tolist()
                if req.marker_id in all_id_list:
                    marker_index = all_id_list.index(req.marker_id)
                    marker_corners = corners[marker_index][0]
                    center = marker_corners.mean(axis=0).astype(int)
                    cv_image = cv2.aruco.drawDetectedMarkers(self.image, corners, ids)
                    cv2.circle(cv_image, tuple(center), 5, (0, 0, 255), -1)
                        
                    u = int(marker_corners[:, 0].mean())
                    v = int(marker_corners[:, 1].mean())
                    self.get_logger().info(f"Marker ID {req.marker_id} found at ({u}, {v})")

                    depth = self.depth_image[v, u]
                    result = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)

                    rvecs, _, _ = cv2.aruco.estimatePoseSingleMarkers(
                        corners, self.marker_length, self.camera_matrix, self.dist_coeffs
                    )
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[marker_index])
                    quad = self.rotation_matrix_to_quaternion(rotation_matrix)
                    trans = [result[0] / 1000.0, result[1] / 1000.0, result[2] / 1000.0]
                    self.publish_transform(trans, quad, f"{ids[marker_index]}_intrinsic")
                    res.pose.position.x = trans[0]
                    res.pose.position.y = trans[1]
                    res.pose.position.z = trans[2]
                    res.pose.orientation.x = quad[0]
                    res.pose.orientation.y = quad[1]
                    res.pose.orientation.z = quad[2]
                    res.pose.orientation.w = quad[3]

                    res.success = True
                    return res
                else:
                    self.get_logger().warn(f"Cant find marker with id: {req.marker_id}")
                    res.success = False
                    return res
            else:
                self.get_logger().warn(f"Cant find marker with id: {req.marker_id}")
                res.success = False
                return res
        except Exception as a:
            self.get_logger().error(a)
            res.success = False
            return res

    def rotation_matrix_to_rpy(self, R):
        # Extract roll, pitch, yaw from the rotation matrix
        sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
        singular = sy < 1e-6

        if not singular:
            roll = np.arctan2(R[2, 1], R[2, 2])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = np.arctan2(R[1, 0], R[0, 0])
        else:
            roll = np.arctan2(-R[1, 2], R[1, 1])
            pitch = np.arctan2(-R[2, 0], sy)
            yaw = 0

        roll = np.degrees(roll)
        pitch = np.degrees(pitch)
        yaw = np.degrees(yaw)
        return (roll, pitch, yaw)

    def rotation_matrix_to_quaternion(self, R):
        # Convert a rotation matrix to a quaternion
        q = np.empty((4,), dtype=np.float64)
        t = np.trace(R)
        if t > 0:
            t = np.sqrt(t + 1.0)
            q[3] = 0.5 * t
            t = 0.5 / t
            q[0] = (R[2, 1] - R[1, 2]) * t
            q[1] = (R[0, 2] - R[2, 0]) * t
            q[2] = (R[1, 0] - R[0, 1]) * t
        else:
            i = 0
            if R[1, 1] > R[0, 0]:
                i = 1
            if R[2, 2] > R[i, i]:
                i = 2
            j = (i + 1) % 3
            k = (j + 1) % 3
            t = np.sqrt(R[i, i] - R[j, j] - R[k, k] + 1.0)
            q[i] = 0.5 * t
            t = 0.5 / t
            q[3] = (R[k, j] - R[j, k]) * t
            q[j] = (R[j, i] + R[i, j]) * t
            q[k] = (R[k, i] + R[i, k]) * t
        return q

    def publish_transform(self, translation, quad, marker_id):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = marker_id

        transform.transform.translation.x = translation[0]
        transform.transform.translation.y = translation[1]
        transform.transform.translation.z = translation[2]

        transform.transform.rotation.x = quad[0]
        transform.transform.rotation.y = quad[1]
        transform.transform.rotation.z = quad[2]
        transform.transform.rotation.w = quad[3]

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    aruco_reader = ArcuoReader()
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(aruco_reader)
    executor.spin()
    executor.shutdown()
    aruco_reader.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
