#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""AprilTag detector for the UAV downward camera.

Subscribes to a raw image topic, detects AprilTags with pyapriltags and
publishes:
  - ~tag_pose (geometry_msgs/PoseStamped): pose of the target tag in the
    camera optical frame (z forward, x right, y down), stamped with the
    image timestamp.
  - TF: <camera_frame> -> tag_<id> for every detected tag.

All external interfaces (topics, tag family/size/id, frames) are parameters.
"""
import numpy as np
import rospy
import tf2_ros
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import CameraInfo, Image

import cv2
import pyapriltags
import tf.transformations as tr


def to_homogeneous(R, t):
    """3x3 rotation + translation vector -> 4x4 homogeneous matrix."""
    m = tr.identity_matrix()
    m[:3, :3] = np.asarray(R)
    m[:3, 3] = np.asarray(t).flatten()
    return m


def hm_to_pose_msg(hm):
    q = tr.quaternion_from_matrix(hm)
    m = PoseStamped()
    m.pose.position.x = float(hm[0, 3])
    m.pose.position.y = float(hm[1, 3])
    m.pose.position.z = float(hm[2, 3])
    m.pose.orientation.x = float(q[0])
    m.pose.orientation.y = float(q[1])
    m.pose.orientation.z = float(q[2])
    m.pose.orientation.w = float(q[3])
    return m


class TagDetector(object):

    def __init__(self):
        self.image_topic = rospy.get_param('~image_topic',
                                          '/iris_downcam0/down_cam/image_raw')
        self.info_topic = rospy.get_param('~camera_info_topic',
                                          '/iris_downcam0/down_cam/camera_info')
        self.tag_family = rospy.get_param('~tag_family', 'tag36h11')
        self.tag_size = float(rospy.get_param('~tag_size', 0.4))
        self.tag_id = int(rospy.get_param('~tag_id', 0))          # -1: any tag
        self.camera_frame = rospy.get_param('~camera_frame',
                                            'down_cam_optical_frame')
        self.max_rate = float(rospy.get_param('~max_rate', 15.0))  # Hz cap
        self.quad_decimate = float(rospy.get_param('~quad_decimate', 1.5))

        self.bridge = CvBridge()
        self.detector = pyapriltags.Detector(
            families=self.tag_family,
            nthreads=4,
            quad_decimate=self.quad_decimate,
            refine_edges=True)

        self.cam_k = None
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.pub_pose = rospy.Publisher('~tag_pose', PoseStamped,
                                        queue_size=5)
        self._min_dt = rospy.Duration(1.0 / self.max_rate) \
            if self.max_rate > 0 else rospy.Duration(0)
        self._last = rospy.Time(0)
        self.info_sub = rospy.Subscriber(self.info_topic, CameraInfo,
                                         self.info_cb, queue_size=1)
        self.image_sub = rospy.Subscriber(self.image_topic, Image,
                                          self.image_cb, queue_size=1,
                                          buff_size=2 ** 24)
        rospy.loginfo('[tag_detector] subscribing image=%s camera_info=%s '
                      '(family=%s size=%.3f id=%d)', self.image_topic,
                      self.info_topic, self.tag_family, self.tag_size,
                      self.tag_id)

    def info_cb(self, msg):
        if msg.K[0] <= 0 or msg.K[4] <= 0:
            rospy.logwarn_throttle(5, '[tag_detector] invalid camera_info on %s'
                                   % self.info_topic)
            return
        cam_k = (msg.K[0], msg.K[4], msg.K[2], msg.K[5])
        if cam_k != self.cam_k:
            self.cam_k = cam_k
            rospy.loginfo('[tag_detector] got camera K: %s', str(self.cam_k))

    def image_cb(self, msg):
        if (msg.header.stamp - self._last) < self._min_dt:
            return
        self._last = msg.header.stamp

        if self.cam_k is None:
            return
        try:
            gray = self.bridge.imgmsg_to_cv2(msg, 'mono8')
        except Exception as e:
            rospy.logwarn_throttle(5, '[tag_detector] cv_bridge: %s' % e)
            return

        dets = self.detector.detect(
            gray, estimate_tag_pose=True, camera_params=self.cam_k,
            tag_size=self.tag_size)

        for d in dets:
            self.send_tf(d, msg.header.stamp)

        chosen = None
        for d in dets:
            if self.tag_id < 0 or d.tag_id == self.tag_id:
                chosen = d
                break
        if chosen is None:
            return
        if chosen.pose_R is None or chosen.pose_t is None:
            return

        hm = to_homogeneous(chosen.pose_R, chosen.pose_t)
        out = hm_to_pose_msg(hm)
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.camera_frame
        self.pub_pose.publish(out)

    def send_tf(self, det, stamp):
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.camera_frame
        t.child_frame_id = 'tag_%d' % det.tag_id
        hm = to_homogeneous(det.pose_R, det.pose_t)
        q = tr.quaternion_from_matrix(hm)
        t.transform.translation.x = float(hm[0, 3])
        t.transform.translation.y = float(hm[1, 3])
        t.transform.translation.z = float(hm[2, 3])
        t.transform.rotation.x = float(q[0])
        t.transform.rotation.y = float(q[1])
        t.transform.rotation.z = float(q[2])
        t.transform.rotation.w = float(q[3])
        self.tf_broadcaster.sendTransform(t)


def main():
    rospy.init_node('tag_detector')
    TagDetector()
    rospy.spin()


if __name__ == '__main__':
    main()
