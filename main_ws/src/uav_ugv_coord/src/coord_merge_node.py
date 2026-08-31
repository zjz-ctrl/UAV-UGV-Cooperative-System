#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Merge the UAV and UGV local coordinate frames using an AprilTag.

The notation ``T_A_B`` means the pose of frame B expressed in frame A.  For
every tag detection the node computes

    T_mU_bG = T_mU_bU * T_bU_cam * T_cam_tag * inverse(T_bG_tag)
    T_mU_mG = T_mU_bG * inverse(T_mG_bG)

where mU/mG are the two local map frames and bU/bG are the vehicle body
frames.  Odometry is interpolated to the image timestamp before evaluating
the chain.

The map-to-map transform is constant while both estimators keep their local
origins, so repeated observations are averaged.  The published UGV pose is
always derived from that same averaged transform and the current UGV
odometry; raw and averaged estimates are never mixed on one topic.
"""
from collections import deque

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String

import tf.transformations as tr


def xyz_rpy_to_matrix(xyz, rpy):
    mat = tr.euler_matrix(rpy[0], rpy[1], rpy[2])
    mat[:3, 3] = np.asarray(xyz, dtype=float)
    return mat


def pose_to_matrix(pose):
    p = pose.position
    q = pose.orientation
    mat = tr.quaternion_matrix([q.x, q.y, q.z, q.w])
    mat[:3, 3] = [p.x, p.y, p.z]
    return mat


def odom_to_matrix(msg):
    return pose_to_matrix(msg.pose.pose)


def interpolate_matrices(first, second, fraction):
    """Linearly interpolate translation and SLERP the orientation."""
    fraction = float(np.clip(fraction, 0.0, 1.0))
    q0 = tr.quaternion_from_matrix(first)
    q1 = tr.quaternion_from_matrix(second)
    quat = tr.quaternion_slerp(q0, q1, fraction)
    mat = tr.quaternion_matrix(quat)
    mat[:3, 3] = ((1.0 - fraction) * first[:3, 3]
                  + fraction * second[:3, 3])
    return mat


def merge_coordinate_frames(T_mU_bU, T_mG_bG, T_bU_cam, T_cam_tag,
                            T_bG_tag):
    """Return ``(T_mU_bG, T_mU_mG)`` for one synchronized observation."""
    T_mU_bG = (T_mU_bU.dot(T_bU_cam).dot(T_cam_tag)
               .dot(np.linalg.inv(T_bG_tag)))
    T_mU_mG = T_mU_bG.dot(np.linalg.inv(T_mG_bG))
    return T_mU_bG, T_mU_mG


def matrix_to_tf(parent, child, mat, stamp):
    quat = tr.quaternion_from_matrix(mat)
    msg = TransformStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = parent
    msg.child_frame_id = child
    msg.transform.translation.x = float(mat[0, 3])
    msg.transform.translation.y = float(mat[1, 3])
    msg.transform.translation.z = float(mat[2, 3])
    msg.transform.rotation.x = float(quat[0])
    msg.transform.rotation.y = float(quat[1])
    msg.transform.rotation.z = float(quat[2])
    msg.transform.rotation.w = float(quat[3])
    return msg


def matrix_to_pose(frame, mat, stamp):
    quat = tr.quaternion_from_matrix(mat)
    msg = PoseStamped()
    msg.header.stamp = stamp
    msg.header.frame_id = frame
    msg.pose.position.x = float(mat[0, 3])
    msg.pose.position.y = float(mat[1, 3])
    msg.pose.position.z = float(mat[2, 3])
    msg.pose.orientation.x = float(quat[0])
    msg.pose.orientation.y = float(quat[1])
    msg.pose.orientation.z = float(quat[2])
    msg.pose.orientation.w = float(quat[3])
    return msg


class OdomBuffer(object):
    """Store recent odometry and interpolate it at a requested timestamp."""

    def __init__(self, name, max_age):
        self.name = name
        self.max_age = rospy.Duration(max_age)
        self.buf = deque(maxlen=300)

    def add(self, msg):
        self.buf.append((msg.header.stamp, odom_to_matrix(msg)))

    def lookup(self, stamp):
        """Return ``(pose, age)`` interpolated at ``stamp`` when possible.

        For an interpolated result, age is the distance to the farther of the
        two bracketing samples.  This prevents interpolation across a large
        odometry gap.  Outside the buffered interval the nearest pose is used
        and age is its absolute timestamp difference.
        """
        if not self.buf:
            raise LookupError('%s: no odometry received yet' % self.name)

        samples = sorted(list(self.buf), key=lambda sample: sample[0].to_sec())
        target = stamp.to_sec()
        before = None
        after = None
        for sample in samples:
            sample_time = sample[0].to_sec()
            if sample_time <= target:
                before = sample
            if sample_time >= target:
                after = sample
                break

        if before is not None and after is not None:
            start = before[0].to_sec()
            end = after[0].to_sec()
            if end > start:
                fraction = (target - start) / (end - start)
                age = max(target - start, end - target)
                return (interpolate_matrices(before[1], after[1], fraction),
                        rospy.Duration(age))
            return before[1], rospy.Duration(0.0)

        nearest = min(samples,
                      key=lambda sample: abs(sample[0].to_sec() - target))
        age = abs(nearest[0].to_sec() - target)
        return nearest[1], rospy.Duration(age)


class TransformAverage(object):
    """Running SE(3) average using a mean translation and quaternion mean."""

    def __init__(self):
        self.count = 0
        self.translation_sum = np.zeros(3)
        self.quaternion_outer_sum = np.zeros((4, 4))

    def add(self, mat):
        quat = tr.quaternion_from_matrix(mat)
        self.count += 1
        self.translation_sum += mat[:3, 3]
        # q and -q produce the same outer product, avoiding sign ambiguity.
        self.quaternion_outer_sum += np.outer(quat, quat)
        _, vectors = np.linalg.eigh(self.quaternion_outer_sum)
        mean_quat = vectors[:, -1]
        if mean_quat[3] < 0.0:
            mean_quat = -mean_quat
        result = tr.quaternion_matrix(mean_quat)
        result[:3, 3] = self.translation_sum / self.count
        return result


class CoordMerge(object):

    def __init__(self):
        self.odom_uav_topic = rospy.get_param(
            '~odom_uav_topic', '/uav0/mavros/local_position/odom')
        self.odom_ugv_topic = rospy.get_param(
            '~odom_ugv_topic', '/ugv1/mavros/local_position/odom')
        self.tag_pose_topic = rospy.get_param(
            '~tag_pose_topic', '/tag_detector/tag_pose')

        self.map_uav_frame = rospy.get_param('~map_uav_frame', 'map_uav')
        self.base_uav_frame = rospy.get_param('~base_uav_frame', 'base_uav')
        self.map_ugv_frame = rospy.get_param('~map_ugv_frame', 'map_ugv')
        self.base_ugv_frame = rospy.get_param('~base_ugv_frame', 'base_ugv')

        # Pose of the camera optical frame in the UAV FLU body frame.
        cam_xyz = rospy.get_param('~cam_xyz', [0.0, 0.0, -0.17])
        cam_rpy = rospy.get_param(
            '~cam_rpy', [3.1415926536, 0.0, -1.5707963268])
        # Pose of pyapriltags' tag frame in the UGV FLU body frame.
        tag_xyz = rospy.get_param('~tag_xyz', [0.0, 0.0, 0.30])
        tag_rpy = rospy.get_param(
            '~tag_rpy', [3.1415926536, 0.0, 1.5707963268])
        self.T_bU_cam = xyz_rpy_to_matrix(cam_xyz, cam_rpy)
        self.T_bG_tag = xyz_rpy_to_matrix(tag_xyz, tag_rpy)

        self.max_pose_age = float(rospy.get_param('~max_pose_age', 0.05))
        self.tf_rate = float(rospy.get_param('~tf_publish_rate', 10.0))

        self.odom_uav = OdomBuffer('uav', self.max_pose_age)
        self.odom_ugv = OdomBuffer('ugv', self.max_pose_age)
        self.map_average = TransformAverage()
        self.last_map_transform = None

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub_map_tf = rospy.Publisher(
            '~map_transform', TransformStamped, queue_size=5, latch=True)
        self.pub_ugv_in_uav = rospy.Publisher(
            '~ugv_pose_in_map_uav', PoseStamped, queue_size=5)
        self.pub_status = rospy.Publisher('~status', String, queue_size=5)

        rospy.Subscriber(self.odom_uav_topic, Odometry, self.uav_odom_cb,
                         queue_size=50)
        rospy.Subscriber(self.odom_ugv_topic, Odometry, self.ugv_odom_cb,
                         queue_size=50)
        rospy.Subscriber(self.tag_pose_topic, PoseStamped, self.tag_cb,
                         queue_size=5)

        rospy.Timer(rospy.Duration(1.0 / self.tf_rate), self.publish_map_tf)
        rospy.loginfo('[coord_merge] ready: uav_odom=%s ugv_odom=%s tags=%s',
                      self.odom_uav_topic, self.odom_ugv_topic,
                      self.tag_pose_topic)

    def uav_odom_cb(self, msg):
        mat = odom_to_matrix(msg)
        self.odom_uav.add(msg)
        self.tf_broadcaster.sendTransform(matrix_to_tf(
            self.map_uav_frame, self.base_uav_frame, mat, msg.header.stamp))

    def ugv_odom_cb(self, msg):
        mat = odom_to_matrix(msg)
        self.odom_ugv.add(msg)
        self.tf_broadcaster.sendTransform(matrix_to_tf(
            self.map_ugv_frame, self.base_ugv_frame, mat, msg.header.stamp))
        if self.last_map_transform is not None:
            self.publish_ugv_pose(mat, msg.header.stamp)

    def publish_ugv_pose(self, T_mG_bG, stamp):
        T_mU_bG = self.last_map_transform.dot(T_mG_bG)
        self.pub_ugv_in_uav.publish(matrix_to_pose(
            self.map_uav_frame, T_mU_bG, stamp))

    def tag_cb(self, msg):
        stamp = msg.header.stamp
        try:
            T_mU_bU, age_u = self.odom_uav.lookup(stamp)
            T_mG_bG, age_g = self.odom_ugv.lookup(stamp)
        except LookupError as exc:
            rospy.logwarn_throttle(2, '[coord_merge] %s' % exc)
            self.pub_status.publish(String(data='NO_ODOMETRY'))
            return
        if age_u > self.odom_uav.max_age or age_g > self.odom_ugv.max_age:
            rospy.logwarn_throttle(
                2, '[coord_merge] odometry too far from image time '
                   '(uav %.3fs ugv %.3fs)',
                age_u.to_sec(), age_g.to_sec())
            self.pub_status.publish(String(data='STALE_ODOMETRY'))
            return

        T_cam_tag = pose_to_matrix(msg.pose)
        _, T_mU_mG = merge_coordinate_frames(
            T_mU_bU, T_mG_bG, self.T_bU_cam, T_cam_tag, self.T_bG_tag)
        self.last_map_transform = self.map_average.add(T_mU_mG)

        self.pub_map_tf.publish(matrix_to_tf(
            self.map_uav_frame, self.map_ugv_frame,
            self.last_map_transform, stamp))

        position = self.last_map_transform[:3, 3]
        self.pub_status.publish(String(
            data=('MERGED samples=%d dt_u=%.3f dt_g=%.3f '
                  'map_ugv=[%.2f %.2f %.2f]')
                 % (self.map_average.count, age_u.to_sec(), age_g.to_sec(),
                    position[0], position[1], position[2])))
        rospy.loginfo_throttle(
            5, '[coord_merge] %s -> %s: t=[%.2f %.2f %.2f], samples=%d',
            self.map_uav_frame, self.map_ugv_frame,
            position[0], position[1], position[2], self.map_average.count)

    def publish_map_tf(self, _event):
        if self.last_map_transform is None:
            return
        # The map-to-map alignment is valid at the current time until either
        # estimator resets its local origin.
        self.tf_broadcaster.sendTransform(matrix_to_tf(
            self.map_uav_frame, self.map_ugv_frame,
            self.last_map_transform, rospy.Time.now()))


def main():
    rospy.init_node('coord_merge')
    CoordMerge()
    rospy.spin()


if __name__ == '__main__':
    main()
