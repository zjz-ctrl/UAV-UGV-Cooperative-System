#!/usr/bin/env python3
"""Track a UGV with visual alignment, incremental odometry prediction, and visual correction."""

import copy
import math

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import concatenate_matrices, inverse_matrix, quaternion_from_euler
from tf.transformations import quaternion_from_matrix, quaternion_matrix, quaternion_slerp, translation_matrix
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


def pose_matrix(pose):
    return concatenate_matrices(
        translation_matrix((pose.position.x, pose.position.y, pose.position.z)),
        quaternion_matrix((pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)),
    )


def matrix_pose(matrix, pose):
    pose.position.x, pose.position.y, pose.position.z = matrix[0][3], matrix[1][3], matrix[2][3]
    quaternion = quaternion_from_matrix(matrix)
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion


def blend_transforms(prediction, measurement, gain):
    translation = (1.0 - gain) * prediction[:3, 3] + gain * measurement[:3, 3]
    rotation = quaternion_slerp(quaternion_from_matrix(prediction), quaternion_from_matrix(measurement), gain)
    return concatenate_matrices(translation_matrix(translation), quaternion_matrix(rotation))


class RelativeTargetEstimator:
    def __init__(self):
        self.uav_odom = None
        self.ugv_odom = None
        self.alignment = None  # iris_0/odom -> ugv_0/odom, initialized only from vision.
        self.last_target_stamp = rospy.Time(0)
        self.last_visual_stamp = rospy.Time(0)
        self.last_tf_stamp = rospy.Time(0)
        self.frame_id = rospy.get_param("~uav_odom_frame", "iris_0/odom")
        self.uav_base_frame = rospy.get_param("~uav_base_frame", "iris_0/base_link")
        self.ugv_odom_frame = rospy.get_param("~ugv_odom_frame", "ugv_0/odom")
        self.camera_frame = rospy.get_param("~camera_frame", "iris_0/camera_link")
        self.max_distance = rospy.get_param("~max_distance", 8.0)
        self.max_innovation = rospy.get_param("~max_innovation", 2.0)
        self.correction_gain = rospy.get_param("~visual_correction_gain", 0.30)
        self.max_prediction_age = rospy.get_param("~max_prediction_age", 1.0)
        base_to_camera = rospy.get_param("~uav_base_to_camera_translation", [0.12, 0.0, 0.015])
        base_to_camera_rpy = rospy.get_param("~uav_base_to_camera_rpy", [0.0, 0.0, 0.0])
        base_to_marker = rospy.get_param("~ugv_base_to_marker_translation", [0.0, 0.0, 0.20])
        base_to_marker_rpy = rospy.get_param("~ugv_base_to_marker_rpy", [0.0, 0.0, 0.0])
        self.base_to_camera = concatenate_matrices(
            translation_matrix(base_to_camera), quaternion_matrix(quaternion_from_euler(*base_to_camera_rpy)))
        self.marker_to_base = inverse_matrix(concatenate_matrices(
            translation_matrix(base_to_marker), quaternion_matrix(quaternion_from_euler(*base_to_marker_rpy))))
        self.pose_pub = rospy.Publisher("/air_ground/relative_target", PoseWithCovarianceStamped, queue_size=10)
        self.valid_pub = rospy.Publisher("/air_ground/relative_target_valid", Bool, queue_size=1, latch=True)
        self.marker_pub = rospy.Publisher("/air_ground/relative_target_marker", Marker, queue_size=1)
        self.icons_pub = rospy.Publisher("/air_ground/vehicle_icons", MarkerArray, queue_size=1)
        self.tf_broadcaster = TransformBroadcaster()
        rospy.Subscriber("/iris_0/mavros/local_position/odom", Odometry, self.uav_odom_callback, queue_size=20)
        # This is the UGV's own incremental odometry broadcast over the cooperation link, not Gazebo truth.
        rospy.Subscriber("/ugv_0/odom", Odometry, self.ugv_odom_callback, queue_size=20)
        rospy.Subscriber("/iris_0/ugv_observation", PoseWithCovarianceStamped, self.observation_callback, queue_size=20)
        rospy.Timer(rospy.Duration(0.05), self.publish_validity)

    def uav_odom_callback(self, message):
        self.uav_odom = message
        transform = TransformStamped()
        transform.header.stamp = message.header.stamp if not message.header.stamp.is_zero() else rospy.Time.now()
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.uav_base_frame
        transform.transform.translation.x = message.pose.pose.position.x
        transform.transform.translation.y = message.pose.pose.position.y
        transform.transform.translation.z = message.pose.pose.position.z
        transform.transform.rotation = message.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)
        self.publish_icons()

    def ugv_odom_callback(self, message):
        self.ugv_odom = message
        self.publish_prediction(message.header.stamp)

    def visual_target_matrix(self, observation):
        return concatenate_matrices(
            pose_matrix(self.uav_odom.pose.pose), self.base_to_camera,
            pose_matrix(observation.pose.pose), self.marker_to_base,
        )

    def observation_callback(self, observation):
        if self.uav_odom is None or self.ugv_odom is None:
            return
        if observation.header.frame_id and observation.header.frame_id != self.camera_frame:
            rospy.logwarn_throttle(2.0, "Ignoring UGV observation in unexpected frame '%s'", observation.header.frame_id)
            return
        stamp = observation.header.stamp if not observation.header.stamp.is_zero() else rospy.Time.now()
        if abs((self.uav_odom.header.stamp - stamp).to_sec()) > 0.15:
            return
        position = observation.pose.pose.position
        if math.sqrt(position.x ** 2 + position.y ** 2 + position.z ** 2) > self.max_distance:
            return
        visual_target = self.visual_target_matrix(observation)
        ugv_pose = pose_matrix(self.ugv_odom.pose.pose)
        if self.alignment is None:
            self.alignment = concatenate_matrices(visual_target, inverse_matrix(ugv_pose))
            rospy.loginfo("Visual UGV alignment initialized in the UAV odometry frame")
        else:
            predicted_target = self.alignment.dot(ugv_pose)
            innovation = math.sqrt(sum((predicted_target[i][3] - visual_target[i][3]) ** 2 for i in range(3)))
            if innovation <= self.max_innovation:
                corrected_target = blend_transforms(predicted_target, visual_target, self.correction_gain)
                self.alignment = corrected_target.dot(inverse_matrix(ugv_pose))
            else:
                rospy.logwarn_throttle(2.0, "Rejected visual UGV correction with %.2f m innovation", innovation)
                return
        self.last_visual_stamp = stamp
        self.publish_prediction(stamp, observation.pose.covariance)

    def publish_prediction(self, stamp=None, covariance=None):
        if self.alignment is None or self.ugv_odom is None:
            return
        stamp = stamp if stamp and not stamp.is_zero() else self.ugv_odom.header.stamp
        if stamp.is_zero():
            stamp = rospy.Time.now()
        target_matrix = self.alignment.dot(pose_matrix(self.ugv_odom.pose.pose))
        target = PoseWithCovarianceStamped()
        target.header.stamp = stamp
        target.header.frame_id = self.frame_id
        matrix_pose(target_matrix, target.pose.pose)
        if covariance is not None:
            target.pose.covariance = covariance
        self.last_target_stamp = stamp
        self.pose_pub.publish(target)
        self.publish_tf(stamp)
        self.publish_marker(target)
        self.publish_icons(target)

    def publish_tf(self, stamp):
        if stamp <= self.last_tf_stamp:
            return
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = self.frame_id
        transform.child_frame_id = self.ugv_odom_frame
        transform.transform.translation.x = self.alignment[0][3]
        transform.transform.translation.y = self.alignment[1][3]
        transform.transform.translation.z = self.alignment[2][3]
        quaternion = quaternion_from_matrix(self.alignment)
        transform.transform.rotation.x, transform.transform.rotation.y = quaternion[0], quaternion[1]
        transform.transform.rotation.z, transform.transform.rotation.w = quaternion[2], quaternion[3]
        self.tf_broadcaster.sendTransform(transform)
        self.last_tf_stamp = stamp

    def publish_validity(self, _event):
        valid = self.alignment is not None and self.ugv_odom is not None
        if valid:
            valid = (rospy.Time.now() - self.ugv_odom.header.stamp).to_sec() <= self.max_prediction_age
        self.valid_pub.publish(Bool(data=valid))

    def publish_marker(self, target):
        marker = Marker()
        marker.header = target.header
        marker.ns = "relative_target"
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose = target.pose.pose
        marker.scale.x = marker.scale.y = 0.45
        marker.scale.z = 0.12
        marker.color.r, marker.color.g, marker.color.a = 1.0, 0.8, 0.9
        self.marker_pub.publish(marker)

    def publish_icons(self, target=None):
        if self.uav_odom is None:
            return
        stamp = rospy.Time.now()
        icons = MarkerArray()
        uav = Marker()
        uav.header.frame_id = self.frame_id
        uav.header.stamp = stamp
        uav.ns = "vehicle_icons"
        uav.id = 0
        uav.type = Marker.ARROW
        uav.action = Marker.ADD
        uav.pose = copy.deepcopy(self.uav_odom.pose.pose)
        uav.scale.x, uav.scale.y, uav.scale.z = 0.75, 0.22, 0.22
        uav.color.b, uav.color.g, uav.color.a = 1.0, 0.8, 0.95
        icons.markers.append(uav)
        uav_label = Marker()
        uav_label.header = uav.header
        uav_label.ns = "vehicle_labels"
        uav_label.id = 0
        uav_label.type = Marker.TEXT_VIEW_FACING
        uav_label.action = Marker.ADD
        uav_label.pose = copy.deepcopy(self.uav_odom.pose.pose)
        uav_label.pose.position.z += 0.35
        uav_label.scale.z = 0.25
        uav_label.color.b, uav_label.color.g, uav_label.color.a = 1.0, 0.8, 1.0
        uav_label.text = "UAV"
        icons.markers.append(uav_label)
        if target is not None:
            ugv = Marker()
            ugv.header = target.header
            ugv.ns = "vehicle_icons"
            ugv.id = 1
            ugv.type = Marker.CUBE
            ugv.action = Marker.ADD
            ugv.pose = copy.deepcopy(target.pose.pose)
            ugv.scale.x, ugv.scale.y, ugv.scale.z = 0.72, 0.48, 0.22
            ugv.color.r, ugv.color.g, ugv.color.b, ugv.color.a = 0.1, 0.45, 1.0, 0.95
            icons.markers.append(ugv)
            ugv_label = Marker()
            ugv_label.header = target.header
            ugv_label.ns = "vehicle_labels"
            ugv_label.id = 1
            ugv_label.type = Marker.TEXT_VIEW_FACING
            ugv_label.action = Marker.ADD
            ugv_label.pose = copy.deepcopy(target.pose.pose)
            ugv_label.pose.position.z += 0.35
            ugv_label.scale.z = 0.25
            ugv_label.color.r, ugv_label.color.g, ugv_label.color.b, ugv_label.color.a = 0.1, 0.8, 1.0, 1.0
            ugv_label.text = "UGV"
            icons.markers.append(ugv_label)
        self.icons_pub.publish(icons)


if __name__ == "__main__":
    rospy.init_node("relative_target_estimator")
    RelativeTargetEstimator()
    rospy.spin()
