#!/usr/bin/env python3
import time
import unittest
import numpy as np
import rospy
import rostest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist


class TransportTest(unittest.TestCase):
    def test_motion_line_loss_and_camera_timeout(self):
        seen = []
        sub = rospy.Subscriber(
            "/test/cmd",
            Twist,
            lambda m: seen.append((time.monotonic(), m.linear.x, m.angular.z)),
        )
        pub = rospy.Publisher("/test/camera", Image, queue_size=1)
        bridge = CvBridge()
        deadline = time.monotonic() + 8
        while pub.get_num_connections() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        self.assertGreater(pub.get_num_connections(), 0)
        image = np.zeros((80, 100, 3), np.uint8)
        image[55:, 70:80] = (0, 255, 255)

        def send(frame):
            start = time.monotonic()
            for _ in range(15):
                pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))
                time.sleep(0.05)
            return [m for m in seen if m[0] >= start]

        moving = send(image)
        self.assertTrue(any(x > 0 and z < 0 for _, x, z in moving))
        time.sleep(0.8)
        self.assertEqual(seen[-1][1:], (0.0, 0.0))
        send(image)
        stopped = send(np.zeros_like(image))
        self.assertTrue(stopped)
        self.assertEqual(stopped[-1][1:], (0.0, 0.0))
        sub.unregister()


if __name__ == "__main__":
    rospy.init_node("lane_transport_test")
    rostest.rosrun("duckie_lane_following", "transport", TransportTest)
