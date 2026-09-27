#!/usr/bin/env python

"""follower_ros.py: Robot will follow the Yellow Line in a track"""

__author__ = "Arjun S Kumar (upstream)"
# Source: https://github.com/arjunskumar/Line-Follower--ROS
# Included in Badr Essefiany\u0027s Duckietown coursework repository.

import rospy
import cv2
import cv_bridge
import numpy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class Follower:
  def __init__(self):
    self.bridge = cv_bridge.CvBridge()
    #cv2.namedWindow("window", 1)
    self.cmd_vel_pub = rospy.Publisher('/cmd_vel',Twist, queue_size=1)
    self.twist = Twist()
    # Create callback dependencies before subscribing: messages may arrive immediately.
    self.image_sub = rospy.Subscriber('/mybot/camera1/image_raw', Image, self.image_callback)
    
  def image_callback(self, msg):
    image = self.bridge.imgmsg_to_cv2(msg,desired_encoding='bgr8')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) 
    # change below lines to map the color you wanted robot to follow
    lower_yellow = numpy.array([ 10,  10,  10])
    upper_yellow = numpy.array([255, 255, 250])
    mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
    
    h, w, d = image.shape
    search_top = (3*h)//4
    search_bot = (3*h)//4 + 20
    mask[0:search_top, 0:w] = 0
    mask[search_bot:h, 0:w] = 0
    M = cv2.moments(mask)
    if M['m00'] > 0:
      cx = int(M['m10']/M['m00'])
      cy = int(M['m01']/M['m00'])
      #cv2.circle(image, (cx, cy), 20, (0,0,255), -1)
      # CONTROL starts
      err = cx - w/2
      self.twist.linear.x = 0.2
      self.twist.angular.z = -float(err) / 100
      self.cmd_vel_pub.publish(self.twist)
      # CONTROL ends
    else:
      # Explicitly replace the previous moving command when the line is lost.
      self.twist = Twist()
      self.cmd_vel_pub.publish(self.twist)
    cv2.imshow("mask",mask)
    cv2.imshow("output", image)
    cv2.waitKey(3)

if __name__ == '__main__':
  rospy.init_node('follower')
  follower = Follower()
  rospy.spin()
# END ALL


