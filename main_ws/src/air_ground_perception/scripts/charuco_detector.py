#!/usr/bin/env python3
"""Publish a quality-gated ChArUco board pose from the rendered nadir image."""

import math

import cv2
import numpy as np
import rospy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_matrix


class CharucoDetector:
    def __init__(self):
        self.bridge = CvBridge()
        self.info = None
        dictionary_name = rospy.get_param("~dictionary", "DICT_5X5_100")
        if not hasattr(cv2.aruco, dictionary_name):
            raise rospy.ROSInitException("Unknown ArUco dictionary {}".format(dictionary_name))
        self.dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dictionary_name))
        self.squares_x = int(rospy.get_param("~squares_x", 7))
        self.squares_y = int(rospy.get_param("~squares_y", 5))
        self.square = float(rospy.get_param("~square_length", 0.075))
        self.marker = float(rospy.get_param("~marker_length", 0.055))
        self.board = cv2.aruco.CharucoBoard_create(
            self.squares_x, self.squares_y, self.square, self.marker, self.dictionary)
        self.minimum_markers = int(rospy.get_param("~minimum_markers", 4))
        self.minimum_corners = int(rospy.get_param("~minimum_corners", 12))
        self.maximum_rmse = float(rospy.get_param("~maximum_reprojection_error_px", 0.8))
        self.pose_pub = rospy.Publisher(rospy.get_param("~observation_topic"), PoseWithCovarianceStamped, queue_size=2)
        self.valid_pub = rospy.Publisher(rospy.get_param("~valid_topic"), Bool, queue_size=1)
        self.debug_pub = rospy.Publisher(rospy.get_param("~debug_topic"), Image, queue_size=1)
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.params.cornerRefinementWinSize = 5
        rospy.Subscriber(rospy.get_param("~camera_info_topic"), CameraInfo, self.info_callback, queue_size=1)
        rospy.Subscriber(rospy.get_param("~image_topic"), Image, self.image_callback, queue_size=1, buff_size=2 ** 24)

    def info_callback(self, message):
        if message.K[0] > 0 and message.K[4] > 0 and message.width and message.height:
            self.info = message

    @staticmethod
    def pose_from_rt(rvec, tvec):
        rotation, _ = cv2.Rodrigues(rvec)
        matrix = np.eye(4)
        matrix[:3, :3] = rotation
        quaternion = quaternion_from_matrix(matrix)
        return tvec.reshape(3), quaternion

    def publish_debug(self, image, corners, ids, charuco_corners=None, accepted=False, text=""):
        debug = image.copy()
        if ids is not None and len(ids):
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
        if charuco_corners is not None and len(charuco_corners):
            cv2.aruco.drawDetectedCornersCharuco(debug, charuco_corners)
        cv2.putText(debug, ("OK " if accepted else "REJECT ") + text, (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 0) if accepted else (0, 0, 255), 2)
        self.debug_pub.publish(self.bridge.cv2_to_imgmsg(debug, encoding="bgr8"))

    def image_callback(self, message):
        if self.info is None or self.info.width != message.width or self.info.height != message.height:
            self.valid_pub.publish(False)
            return
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as error:
            rospy.logwarn_throttle(2.0, "ChArUco image conversion failed: %s", error)
            self.valid_pub.publish(False)
            return
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.params)
        if ids is None or len(ids) < self.minimum_markers:
            self.publish_debug(image, corners, ids, text="markers={}".format(0 if ids is None else len(ids)))
            self.valid_pub.publish(False)
            return
        cv2.aruco.refineDetectedMarkers(gray, self.board, corners, ids, rejected)
        count, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, self.board, cameraMatrix=np.array(self.info.K).reshape(3, 3),
            distCoeffs=np.array(self.info.D, dtype=np.float64))
        if charuco_ids is None or count < self.minimum_corners:
            self.publish_debug(image, corners, ids, charuco_corners, text="corners={}".format(int(count)))
            self.valid_pub.publish(False)
            return
        camera = np.array(self.info.K, dtype=np.float64).reshape(3, 3)
        distortion = np.array(self.info.D, dtype=np.float64)
        ok, rvec, tvec = cv2.aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self.board, camera, distortion, None, None)
        if not ok or float(tvec[2]) <= 0.0:
            self.publish_debug(image, corners, ids, charuco_corners, text="pnp")
            self.valid_pub.publish(False)
            return
        object_points = np.array([self.board.chessboardCorners[index[0]] for index in charuco_ids], dtype=np.float64)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, camera, distortion)
        residuals = np.linalg.norm(projected.reshape(-1, 2) - charuco_corners.reshape(-1, 2), axis=1)
        rmse = float(math.sqrt(np.mean(residuals ** 2)))
        if not math.isfinite(rmse) or rmse > self.maximum_rmse:
            self.publish_debug(image, corners, ids, charuco_corners, text="rmse={:.2f}".format(rmse))
            self.valid_pub.publish(False)
            return
        translation, quaternion = self.pose_from_rt(rvec, tvec)
        result = PoseWithCovarianceStamped()
        result.header = message.header
        result.header.frame_id = self.info.header.frame_id or message.header.frame_id
        result.pose.pose.position.x, result.pose.pose.position.y, result.pose.pose.position.z = translation
        result.pose.pose.orientation.x, result.pose.pose.orientation.y, result.pose.pose.orientation.z, result.pose.pose.orientation.w = quaternion
        variance = max(1e-6, (rmse * float(translation[2]) / camera[0, 0]) ** 2)
        result.pose.covariance[0] = result.pose.covariance[7] = result.pose.covariance[14] = variance
        result.pose.covariance[35] = max(1e-6, (rmse / max(1.0, len(charuco_ids))) ** 2)
        self.pose_pub.publish(result)
        self.valid_pub.publish(True)
        self.publish_debug(image, corners, ids, charuco_corners, True, "n={} rmse={:.2f}".format(int(count), rmse))


if __name__ == "__main__":
    rospy.init_node("charuco_detector")
    CharucoDetector()
    rospy.spin()
