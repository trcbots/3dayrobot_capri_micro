#!/usr/bin/env python

import rospy
import cv2

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from target_finder import *
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
class target_finder():
	def __init__(self):
		self.bridge = CvBridge()
		self.target_finder = targetfinder()
		self.camera_sub = rospy.Subscriber("/zed/left/image_rect_color", Image, self.callback)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.state_pub = rospy.Publisher('target_state', Bool, queue_size=1)
                self.skip = 0
                self.tracked_count = 0
                self.tracking = False
	def convert2cv(self, camera_img):
		cv_img = self.bridge.imgmsg_to_cv2(camera_img, "bgr8")
		return cv_img


	def callback(self, msg):
                self.skip += 1
                if self.skip % 10 != 0: # reduce process rate
                    return
                self.skip = 0
		cv_img = self.convert2cv(msg)
		angle = self.target_finder.find_target(cv_img)
		if angle is not None:
                    self.tracked_count += 1
                    if self.tracked_count> 5: 
                        self.tracking = True
                else:
                    self.tracked_count = 0
                    self.tracking = False
                
                state_cmd = Bool()
                state_cmd.data = False
                if self.tracking:
                    vel_cmd = Twist()
                    vel_cmd.angular.z = angle
                    self.vel_pub.publish(vel_cmd)
                    state_cmd.data = True
                    self.state_pub.publish(state_cmd)
                    print angle
                else:
                    self.state_pub.publish(state_cmd)
		#self.angle_pub.publish(angle)
		
if __name__ == "__main__":
	tf = target_finder()
	rospy.init_node("target_finder", anonymous=True)
	rospy.spin()
	
	cv2.destroyAllWindows()
