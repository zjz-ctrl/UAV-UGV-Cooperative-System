#!/usr/bin/env python3

import io
import unittest

import rospy
from rospy.msg import serialize_message

from air_ground_coordinate_transform.msg import RegistrationUpdate


class RegistrationUpdateSerializationTest(unittest.TestCase):
    def test_transport_sequence_cannot_change_explicit_revision_or_pose(self):
        message = RegistrationUpdate()
        message.header.seq = 7
        message.header.stamp = rospy.Time.from_sec(12.5)
        message.header.frame_id = "air_ground_origin"
        message.revision = 42
        message.pose.pose.position.x = 1.25
        message.pose.pose.position.y = -2.5
        message.pose.pose.orientation.z = 0.5
        message.pose.pose.orientation.w = 0.8660254037844386
        message.pose.covariance = [0.01 * index for index in range(36)]

        buffer = io.BytesIO()
        serialize_message(buffer, 91, message)
        received = RegistrationUpdate().deserialize(buffer.getvalue()[4:])

        self.assertEqual(received.header.seq, 91)
        self.assertEqual(received.header.stamp, rospy.Time.from_sec(12.5))
        self.assertEqual(received.header.frame_id, "air_ground_origin")
        self.assertEqual(received.revision, 42)
        self.assertEqual(received.pose.pose.position.x, 1.25)
        self.assertEqual(received.pose.pose.position.y, -2.5)
        self.assertEqual(received.pose.pose.orientation.z, 0.5)
        self.assertEqual(received.pose.pose.orientation.w, 0.8660254037844386)
        self.assertEqual(received.pose.covariance, tuple(0.01 * index for index in range(36)))


if __name__ == "__main__":
    unittest.main()
