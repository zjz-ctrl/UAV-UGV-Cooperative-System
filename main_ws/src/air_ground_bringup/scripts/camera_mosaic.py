#!/usr/bin/env python3
"""Publish the UAV and UGV camera feeds as one RViz-friendly image."""

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image


class CameraMosaic:
    def __init__(self):
        self.bridge = CvBridge()
        self.uav_image = None
        self.ugv_image = None
        self.publisher = rospy.Publisher("/air_ground/camera_mosaic", Image, queue_size=1)
        rospy.Subscriber("/iris_0/realsense/depth_camera/color/image_raw", Image, self.uav_callback, queue_size=1)
        rospy.Subscriber("/ugv_0/camera/image_raw", Image, self.ugv_callback, queue_size=1)

    def uav_callback(self, message):
        self.uav_image = message
        self.publish()

    def ugv_callback(self, message):
        self.ugv_image = message
        self.publish()

    def publish(self):
        if self.uav_image is None or self.ugv_image is None:
            return
        try:
            uav = self.bridge.imgmsg_to_cv2(self.uav_image, desired_encoding="bgr8")
            ugv = self.bridge.imgmsg_to_cv2(self.ugv_image, desired_encoding="bgr8")
        except CvBridgeError as error:
            rospy.logwarn_throttle(2.0, "Cannot compose camera mosaic: %s", error)
            return
        height, width = 360, 640
        uav = cv2.resize(uav, (width, height), interpolation=cv2.INTER_AREA)
        ugv = cv2.resize(ugv, (width, height), interpolation=cv2.INTER_AREA)
        cv2.putText(uav, "UAV CAMERA", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        cv2.putText(ugv, "UGV CAMERA", (16, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        mosaic = cv2.hconcat((uav, ugv))
        output = self.bridge.cv2_to_imgmsg(mosaic, encoding="bgr8")
        output.header.stamp = rospy.Time.now()
        self.publisher.publish(output)


if __name__ == "__main__":
    rospy.init_node("camera_mosaic")
    CameraMosaic()
    rospy.spin()
