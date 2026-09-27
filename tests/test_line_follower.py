"""Focused image callback checks; ROS transport and GUI are stubbed."""
import importlib.util
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import cv2
import numpy as np


class LineFollowerTests(unittest.TestCase):
    def setUp(self):
        rospy = ModuleType("rospy")
        rospy.init_node = Mock()
        rospy.spin = Mock()
        rospy.Subscriber = Mock()
        self.publisher = Mock()
        rospy.Publisher = Mock(return_value=self.publisher)
        bridge = ModuleType("cv_bridge")
        bridge.CvBridge = lambda: SimpleNamespace(imgmsg_to_cv2=lambda msg, **kw: msg)
        sensor = ModuleType("sensor_msgs")
        sensor.msg = ModuleType("sensor_msgs.msg")
        sensor.msg.Image = type("Image", (), {})
        sensor.msg.CameraInfo = type("CameraInfo", (), {})
        geometry = ModuleType("geometry_msgs")
        geometry.msg = ModuleType("geometry_msgs.msg")
        geometry.msg.Twist = lambda: SimpleNamespace(
            linear=SimpleNamespace(x=0.0), angular=SimpleNamespace(z=0.0))
        modules = {"rospy": rospy, "cv_bridge": bridge,
                   "sensor_msgs": sensor, "sensor_msgs.msg": sensor.msg,
                   "geometry_msgs": geometry, "geometry_msgs.msg": geometry.msg}
        with patch.dict(sys.modules, modules):
            path = Path(__file__).resolve().parents[1] / "packages" / "LineFollower.py"
            spec = importlib.util.spec_from_file_location("line_follower_under_test", path)
            self.module = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(self.module)
        self.assertFalse(rospy.init_node.called)
        self.assertFalse(rospy.spin.called)
        self.follower = self.module.Follower()

    def process(self, image):
        with patch.object(cv2, "imshow"), patch.object(cv2, "waitKey"):
            self.follower.image_callback(image)

    def test_centroid_steers_toward_line(self):
        for x, expected in [(40, 0.4), (80, 0.0), (120, -0.4)]:
            with self.subTest(x=x):
                self.publisher.reset_mock()
                image = np.zeros((120, 160, 3), dtype=np.uint8)
                # Yellow below the inherited V ceiling of 250.
                image[90:110, x-2:x+3] = (0, 200, 200)
                self.process(image)
                self.publisher.publish.assert_called_once()
                command = self.publisher.publish.call_args.args[0]
                self.assertAlmostEqual(command.linear.x, 0.2)
                self.assertAlmostEqual(command.angular.z, expected)

    def test_pixels_outside_strip_are_ignored(self):
        image = np.zeros((121, 160, 3), dtype=np.uint8)
        image[:85, 30:40] = (0, 200, 200)
        image[112:, 30:40] = (0, 200, 200)
        self.process(image)
        self.publisher.publish.assert_called_once()
        self.assertEqual(self.publisher.publish.call_args.args[0].linear.x, 0.0)

    def test_lost_line_replaces_previous_movement_with_stop(self):
        image = np.zeros((120, 160, 3), dtype=np.uint8)
        image[90:110, 38:43] = (0, 200, 200)
        self.process(image)
        self.assertGreater(self.publisher.publish.call_args.args[0].linear.x, 0)
        self.publisher.reset_mock()
        self.process(np.zeros((120, 160, 3), dtype=np.uint8))
        self.publisher.publish.assert_called_once()
        command = self.publisher.publish.call_args.args[0]
        self.assertEqual(command.linear.x, 0.0)
        self.assertEqual(command.angular.z, 0.0)


if __name__ == "__main__":
    unittest.main()


