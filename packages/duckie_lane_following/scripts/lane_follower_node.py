#!/usr/bin/env python3
"""Publish bounded Twist commands on a compatible robot velocity interface."""

import threading
import time
import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from duckie_lane_following.control import LaneController, CommandState


class LaneFollower:
    def __init__(self):
        self.lock = threading.Lock()
        self.bridge = CvBridge()
        self.controller = LaneController(
            speed=rospy.get_param("~speed", 0.15),
            gain=rospy.get_param("~gain", 0.8),
            max_turn=rospy.get_param("~max_turn", 1.0),
            lower=rospy.get_param("~hsv_lower", [20, 80, 80]),
            upper=rospy.get_param("~hsv_upper", [40, 255, 255]),
            roi_start=rospy.get_param("~roi_start", 0.65),
            min_pixels=rospy.get_param("~min_pixels", 20),
        )
        self.state = CommandState(rospy.get_param("~frame_timeout", 0.5))
        self.publisher = rospy.Publisher(
            rospy.get_param("~cmd_topic", "/cmd_vel"), Twist, queue_size=1
        )
        self.subscriber = rospy.Subscriber(
            rospy.get_param("~image_topic", "/camera/image_raw"),
            Image,
            self.on_image,
            queue_size=1,
            buff_size=2**24,
        )
        self.timer = rospy.Timer(rospy.Duration(0.05), self.on_timer)
        rospy.on_shutdown(self.stop)

    def on_image(self, message):
        received = time.monotonic()
        try:
            command = self.controller.command(
                self.bridge.imgmsg_to_cv2(message, "bgr8")
            )
        except (CvBridgeError, ValueError, cv2.error) as error:
            rospy.logwarn_throttle(5.0, "Invalid camera frame: %s", str(error))
            command = (0.0, 0.0)
        with self.lock:
            self.state.update(command, received)

    def on_timer(self, _event):
        with self.lock:
            speed, turn = self.state.current(time.monotonic())
        message = Twist()
        message.linear.x, message.angular.z = speed, turn
        self.publisher.publish(message)

    def stop(self):
        self.timer.shutdown()
        self.publisher.publish(Twist())


if __name__ == "__main__":
    rospy.init_node("lane_follower")
    LaneFollower()
    rospy.spin()
