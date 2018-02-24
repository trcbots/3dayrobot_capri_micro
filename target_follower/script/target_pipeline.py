#!/usr/bin/env python

import rospy
import cv2

from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from target_finder import *
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from math import atan
class target_finder():
	def __init__(self):
		self.bridge = CvBridge()
		self.target_finder = targetfinder( display_windows=True)
		self.camera_sub = rospy.Subscriber("/zed/left/image_rect_color", Image, self.callback)
		self.depth_sub = rospy.Subscriber("/zed/depth/depth_registered", Image, self.callbackDepth)
		self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
		self.state_pub = rospy.Publisher('target_state', Bool, queue_size=1)
                self.skip = 0
                self.skipD = 0
                self.tracked_count = 0
                self.tracking = False
                self.pixel_ = None
                self.depth_ = None
                self.previous_depth = 1000
                self.cx_ = 656.138
                self.fx_ = 699.465
	def convert2cv(self, camera_img):
		cv_img = self.bridge.imgmsg_to_cv2(camera_img, "bgr8")
		return cv_img

        def callbackDepth(self,msg):
                self.skipD += 1
                if self.skipD % 10 != 0: # reduce process rate
                    return
                self.skipD = 0 
                print("Load depth")
                self.depth_ = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
	def callback(self, msg):
                self.skip += 1
                if self.skip % 10 != 0: # reduce process rate
                    return
                self.skip = 0
		cv_img = self.convert2cv(msg)
		self.pixel_ = self.target_finder.find_target(cv_img)
		if self.pixel_ is not None:
                    self.tracked_count += 1
                    print("The center pixel is at" + str(self.pixel_))
                    #print("Value of the pixel: ")
                    #print(self.depth_[self.pixel_[0],self.pixel_[1]])
                    cur_depth = None
                    if self.depth_ is not None:
                        cur_depth=self.depth_[self.pixel_[1],self.pixel_[0]]
                    if cur_depth is not None and cur_depth > 0:
                        self.previous_depth = cur_depth
                    if self.tracked_count> 5: 
                        self.tracking = True
                    print(self.previous_depth)
                    px = -(self.pixel_[0]-self.cx_)*self.previous_depth/self.fx_    
                    print(px)
                    
                    angle = atan(px/self.previous_depth)
                    print("angle:"+str(angle))
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
