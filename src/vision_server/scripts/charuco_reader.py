#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from bin_packing_msgs.srv import *
from bin_packing_msgs.msg import *
from std_srvs.srv import *
import rclpy.wait_for_message
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import cv2
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import numpy as np
import tf2_ros
import pyrealsense2 as rs
import copy
import time
from icecream import ic
from tf_transformations import euler_from_matrix, euler_matrix

ic.configureOutput(includeContext=True)

TOTAL_CHARUCO = 3  # id 0, 1, 2
FORCE_RP_ZERO = True


class ChArUcoReader(Node):

    def __init__(self):
        super().__init__("charuco_reader")
        self.image_publisher = self.create_publisher(Image, "charuco_reader_output", 10)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.intrinsics = None
        self.image = None
        self.depth_image = None
        self.pointcloud = None
        self.frame_before_fm_placing = None
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)

        self.all_dict_list = [self.aruco_dict]
        for i in range(1, TOTAL_CHARUCO + 1):
            temp_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
            temp_dict.bytesList = temp_dict.bytesList[4 * i :, :, :]
            # temp_dict.bytesList = temp_dict.bytesList[8 * i :, :, :]
            self.all_dict_list.append(temp_dict)

        # self.charuco_board = cv2.aruco.CharucoBoard((9, 5), 0.03, 0.022, self.aruco_dict)
        self.charuco_board = cv2.aruco.CharucoBoard((3, 3), 0.025, 0.018, self.aruco_dict)
        # self.charuco_board = cv2.aruco.CharucoBoard((4, 4), 0.017, 0.012, self.aruco_dict)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.create_service(
            GetAllChArUco, "get_all_charuco", self.get_all_charuco_cb, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_service(
            GetChArUcoPose,
            "get_charuco_pose",
            self.get_charuco_pose_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        
        self.create_service(
            Trigger,
            "save_fm_frame",
            self.save_fm_frame_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )
        
        self.create_service(
            Trigger,
            "get_item_from_fm_cb",
            self.save_fm_frame_cb,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/color/camera_info", self.camera_info_cb, 10
        )
        self.depth_camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera/aligned_depth_to_color/camera_info", self.depth_camera_info_cb, 10
        )
        self.create_subscription(
            Image, "/camera/camera/color/image_raw", self.image_cb, 10, callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_image_cb,
            10,
            callback_group=MutuallyExclusiveCallbackGroup(),
        )

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
        if self.frame_before_fm_placing is None:
            return
        try:
            cv_image = copy.deepcopy(self.image)
            lower_thresh = np.array([0, 17, 83])
            upper_thresh = np.array([24, 255, 255])
            difference = cv2.subtract(self.frame_before_fm_placing, cv_image)
            hsv_image = cv2.cvtColor(difference, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv_image, lower_thresh, upper_thresh)
            
            filtered_image = cv2.bitwise_and(difference, difference, mask=mask)
            gray_diff = cv2.cvtColor(filtered_image, cv2.COLOR_BGR2GRAY)
            _, thresh_img = cv2.threshold(gray_diff, 0, 255, cv2.THRESH_BINARY)
            
            
            contours, _ = cv2.findContours(thresh_img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(cv_image, contours, -1, (0, 0, 255), 2)
            for contour in contours:
                if cv2.contourArea(contour) >= 500:
                    # Get the rotated rectangle
                    rect = cv2.minAreaRect(contour)
                    box = cv2.boxPoints(rect)  # Get the 4 points of the rectangle
                    box = np.int0(box)  # Convert points to integers
                    
                    # Draw the bounding box
                    cv2.drawContours(cv_image, [box], 0, (0, 255, 0), 2)  # Draw in green
                    
                    # Draw the center
                    center = (int(rect[0][0]), int(rect[0][1]))
                    cv2.circle(cv_image, center, 5, (0, 0, 255), -1)  # Draw center in red
                    
                    # Annotate orientation
                    angle = rect[2]
                    if angle < -45:
                        angle += 90
                    elif angle > 45:
                        angle -= 90
                    cv2.putText(cv_image, f'Angle: {angle:.2f}', (center[0] - 50, center[1] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            
                
                
                
                
            fm_pick_frame = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            fm_pick_frame.header.frame_id = "camera_color_optical_frame"
            fm_pick_frame.header.stamp = self.get_clock().now().to_msg()
            self.image_publisher.publish(fm_pick_frame)
        except:
            pass
        """
        cv_image = copy.deepcopy(self.image)
        charuco_found = []
        for index, aruco_dict in enumerate(self.all_dict_list):
            corners, ids, _ = cv2.aruco.detectMarkers(self.image, aruco_dict)
            if ids is not None:
                retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, self.image, self.charuco_board
                )
                if charuco_corners is not None and charuco_ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners,
                        charuco_ids,
                        self.charuco_board,
                        self.camera_matrix,
                        self.dist_coeffs,
                        np.empty(1),
                        np.empty(1),
                    )

                    if success:
                        charuco_frame_offset = np.array(
                            [
                                self.charuco_board.getChessboardSize()[0] * self.charuco_board.getSquareLength() / 2,
                                self.charuco_board.getChessboardSize()[1] * self.charuco_board.getSquareLength() / 2,
                                0,
                            ]
                        )
                        rmat, _ = cv2.Rodrigues(rvec)
                        tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03, 2)

                        
                        # Compute the centroid of the ChArUco corners
                        origin_point = np.float32([[0, 0, 0]])
                        image_points, _ = cv2.projectPoints(origin_point, rvec, tvec, self.camera_matrix, self.dist_coeffs)
                        (u, v) = image_points[0].ravel()
                        center_x = int(u)
                        center_y = int(v)

                        # Draw the centroid on the image
                        cv2.circle(cv_image, (int(center_x), int(center_y)), 3, (255, 255, 0), -1)
                        cv2.putText(
                            cv_image,
                            f"Charuco {index}: ({int(center_x)}, {int(center_y)})",
                            (int(center_x) + 10, int(center_y) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )

                        charuco_found.append(index)

        print(f"charuco_found: {charuco_found}")

        charuco_reader_output = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        charuco_reader_output.header.frame_id = "camera_color_optical_frame"
        charuco_reader_output.header.stamp = self.get_clock().now().to_msg()
        self.image_publisher.publish(charuco_reader_output)
        """

    def get_all_charuco_cb(self, req, res):
        # read the camera image and find all charuco markers
        self.get_logger().info("Got a request to get all charuco markers")
        frame = copy.deepcopy(self.image)
        try:
            cv_image = copy.deepcopy(frame)
            charuco_found = []
            for index, aruco_dict in enumerate(self.all_dict_list):
                corners, ids, _ = cv2.aruco.detectMarkers(frame, aruco_dict)
                if ids is not None:
                    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        corners, ids, frame, self.charuco_board
                    )
                    if charuco_corners is not None and charuco_ids is not None:
                        cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                        success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                            charuco_corners,
                            charuco_ids,
                            self.charuco_board,
                            self.camera_matrix,
                            self.dist_coeffs,
                            np.empty(1),
                            np.empty(1),
                        )

                        if success:
                            charuco_frame_offset = np.array(
                                [
                                    self.charuco_board.getChessboardSize()[0]
                                    * self.charuco_board.getSquareLength()
                                    / 2,
                                    self.charuco_board.getChessboardSize()[1]
                                    * self.charuco_board.getSquareLength()
                                    / 2,
                                    0,
                                ]
                            )
                            rmat, _ = cv2.Rodrigues(rvec)
                            tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)
                            cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03, 2)

                            # Compute the centroid of the ChArUco corners
                            origin_point = np.float32([[0, 0, 0]])
                            image_points, _ = cv2.projectPoints(
                                origin_point, rvec, tvec, self.camera_matrix, self.dist_coeffs
                            )
                            (u, v) = image_points[0].ravel()
                            center_x = int(u)
                            center_y = int(v)

                            # Draw the centroid on the image
                            cv2.circle(cv_image, (int(center_x), int(center_y)), 3, (255, 255, 0), -1)
                            cv2.putText(
                                cv_image,
                                f"Charuco {index}: ({int(center_x)}, {int(center_y)})",
                                (int(center_x) + 10, int(center_y) - 10),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.5,
                                (0, 255, 0),
                                1,
                                cv2.LINE_AA,
                            )
                            charuco_found.append(index)

            charuco_reader_output = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            charuco_reader_output.header.frame_id = "camera_color_optical_frame"
            charuco_reader_output.header.stamp = self.get_clock().now().to_msg()
            self.image_publisher.publish(charuco_reader_output)

            if charuco_found:
                res.charuco_markers = charuco_found
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

    def get_charuco_pose_cb(self, req, res):
        # read the camera image, find the point of the specified charuco id
        self.get_logger().info("Got a request to get charuco point2")
        try:
            position_accumulator = np.zeros(3)
            quaternion_accumulator = np.zeros(4)
            valid_samples = 0

            for i in range(10):
                frame = copy.deepcopy(self.image)
                cv_image = copy.deepcopy(frame)
                corners, ids, _ = cv2.aruco.detectMarkers(frame, self.all_dict_list[req.charuco_id])

                if ids is not None:
                    retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                        corners, ids, frame, self.charuco_board
                    )
                    if charuco_corners is not None and charuco_ids is not None:
                        cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                        success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                            charuco_corners,
                            charuco_ids,
                            self.charuco_board,
                            self.camera_matrix,
                            self.dist_coeffs,
                            np.empty(1),
                            np.empty(1),
                        )

                        if success:
                            charuco_frame_offset = np.array(
                                [
                                    self.charuco_board.getChessboardSize()[0]
                                    * self.charuco_board.getSquareLength()
                                    / 2,
                                    self.charuco_board.getChessboardSize()[1]
                                    * self.charuco_board.getSquareLength()
                                    / 2,
                                    0,
                                ]
                            )
                            rmat, _ = cv2.Rodrigues(rvec)
                            tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)

                            # Compute the centroid of the ChArUco corners
                            origin_point = np.float32([[0, 0, 0]])
                            image_points, _ = cv2.projectPoints(
                                origin_point, rvec, tvec, self.camera_matrix, self.dist_coeffs
                            )
                            (u, v) = image_points[0].ravel()
                            u = int(u)
                            v = int(v)

                            depth = self.depth_image[v, u]
                            result = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)
                            rotation_matrix, _ = cv2.Rodrigues(rvec)
                            if FORCE_RP_ZERO:
                                roll, pitch, yaw = euler_from_matrix(rotation_matrix)
                                ic(roll, pitch, yaw)
                                roll = 0
                                pitch = 0
                                rmat = euler_matrix(roll, pitch, yaw)[:3, :3]
                                quad = self.rotation_matrix_to_quaternion(rmat)
                            else:
                                quad = self.rotation_matrix_to_quaternion(rotation_matrix)
                            trans = [result[0] / 1000.0, result[1] / 1000.0, result[2] / 1000.0]

                            position_accumulator += trans
                            quaternion_accumulator += quad
                            valid_samples += 1
                            ic(trans, quad)
                time.sleep(0.05)

            if valid_samples > 5:
                ic(valid_samples)
                averaged_position = position_accumulator / valid_samples
                averaged_quaternion = quaternion_accumulator / valid_samples
                averaged_quaternion /= np.linalg.norm(averaged_quaternion)  # Normalize the quaternion

                ic(averaged_position, averaged_quaternion)
                self.publish_transform(averaged_position, averaged_quaternion, f"Charuco {req.charuco_id}_intrinsic")
                res.pose.position.x = averaged_position[0]
                res.pose.position.y = averaged_position[1]
                res.pose.position.z = averaged_position[2]
                res.pose.orientation.x = averaged_quaternion[0]
                res.pose.orientation.y = averaged_quaternion[1]
                res.pose.orientation.z = averaged_quaternion[2]
                res.pose.orientation.w = averaged_quaternion[3]
                res.success = True

            else:
                self.get_logger().warn(
                    f"Unable to find a valid marker for more than 5 attempts for ID: {req.charuco_id}"
                )
                res.success = False

            return res

        except Exception as e:
            self.get_logger().error(f"Error: {str(e)}")
            res.success = False
            return res

    def get_charuco_pose_cb_non_average(self, req, res):
        # read the camera image, find the point of the specified charuco id
        self.get_logger().info("Got a request to get charuco point")
        frame = copy.deepcopy(self.image)
        try:
            cv_image = copy.deepcopy(frame)
            corners, ids, _ = cv2.aruco.detectMarkers(frame, self.all_dict_list[req.charuco_id])
            if ids is not None:
                retval, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                    corners, ids, frame, self.charuco_board
                )
                if charuco_corners is not None and charuco_ids is not None:
                    cv2.aruco.drawDetectedCornersCharuco(cv_image, charuco_corners, charuco_ids)
                    success, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
                        charuco_corners,
                        charuco_ids,
                        self.charuco_board,
                        self.camera_matrix,
                        self.dist_coeffs,
                        np.empty(1),
                        np.empty(1),
                    )

                    if success:
                        charuco_frame_offset = np.array(
                            [
                                self.charuco_board.getChessboardSize()[0] * self.charuco_board.getSquareLength() / 2,
                                self.charuco_board.getChessboardSize()[1] * self.charuco_board.getSquareLength() / 2,
                                0,
                            ]
                        )
                        rmat, _ = cv2.Rodrigues(rvec)
                        tvec += (np.dot(rmat, charuco_frame_offset)).reshape(-1, 1)
                        cv2.drawFrameAxes(cv_image, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.03, 2)

                        # Compute the centroid of the ChArUco corners
                        origin_point = np.float32([[0, 0, 0]])
                        image_points, _ = cv2.projectPoints(
                            origin_point, rvec, tvec, self.camera_matrix, self.dist_coeffs
                        )
                        (u, v) = image_points[0].ravel()
                        u = int(u)
                        v = int(v)
                        self.get_logger().info(f"Marker ID {req.charuco_id} found at ({u}, {v})")

                        # Draw the centroid on the image
                        cv2.circle(cv_image, (u, v), 3, (255, 255, 0), -1)
                        cv2.putText(
                            cv_image,
                            f"ChArUco {req.charuco_id}: ({u}, {v})",
                            (u + 10, v - 10),
                            cv2.FONT_HERSHEY_SIMPLEX,
                            0.5,
                            (0, 255, 0),
                            1,
                            cv2.LINE_AA,
                        )

                        depth = self.depth_image[v, u]
                        result = rs.rs2_deproject_pixel_to_point(self.intrinsics, [u, v], depth)

                        rotation_matrix, _ = cv2.Rodrigues(rvec)
                        quad = self.rotation_matrix_to_quaternion(rotation_matrix)
                        trans = [result[0] / 1000.0, result[1] / 1000.0, result[2] / 1000.0]
                        self.publish_transform(trans, quad, f"Charuco {req.charuco_id}_intrinsic")
                        res.pose.position.x = trans[0]
                        res.pose.position.y = trans[1]
                        res.pose.position.z = trans[2]
                        res.pose.orientation.x = quad[0]
                        res.pose.orientation.y = quad[1]
                        res.pose.orientation.z = quad[2]
                        res.pose.orientation.w = quad[3]
                        res.success = True

                        charuco_reader_output = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
                        charuco_reader_output.header.frame_id = "camera_color_optical_frame"
                        charuco_reader_output.header.stamp = self.get_clock().now().to_msg()
                        self.image_publisher.publish(charuco_reader_output)
                        return res

                    else:
                        self.get_logger().warn(f"Cant find marker with id: {req.charuco_id}")
                        res.success = False
                        return res
                else:
                    self.get_logger().warn(f"Cant find marker with id: {req.charuco_id}")
                    res.success = False
                    return res
            else:
                self.get_logger().warn(f"Cant find marker with id: {req.charuco_id}")
                res.success = False
                return res
        except Exception as a:
            self.get_logger().error(a)
            res.success = False
            return res

    def save_fm_frame_cb(self, req, res):
        self.get_logger().info("Receive request to save frame before FM placing")
        # self.frame_before_fm_placing = cv2.cvtColor(copy.deepcopy(self.image), cv2.COLOR_BGR2GRAY)
        self.frame_before_fm_placing = copy.deepcopy(self.image)
        res.success = True
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

    def publish_transform(self, translation, quad, charuco_id):
        transform = TransformStamped()

        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "camera_color_optical_frame"
        transform.child_frame_id = charuco_id

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
    charuco_reader = ChArUcoReader()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
    executor.add_node(charuco_reader)
    executor.spin()
    executor.shutdown()
    charuco_reader.destory_node()

    rclpy.shutdown()


def gen_marker():
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
    # aruco_dict.bytesList = aruco_dict.bytesList[4:, :, :]
    board = cv2.aruco.CharucoBoard((4, 4), 0.017, 0.012, aruco_dict)
    board_image = board.generateImage((1000, 1000), None, 10, 1)
    cv2.imshow("charuco", board_image)
    cv2.waitKey(0)


if __name__ == "__main__":
    # gen_marker()
    main()
