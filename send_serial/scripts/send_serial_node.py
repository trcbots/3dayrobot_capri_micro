#!/usr/bin/env python
# license removed for brevity
import rospy
from python_to_arduino import ArduinoMap
from geometry_msgs.msg import Twist


arduinomap = ArduinoMap("ttyACM0",9600)
# From output ref 1 to 100 percent
ScalingThrottle = 20

def callback(data):
    arduinomap.arduinoSerial(data.angular.z*100+70, data.linear.x*ScalingThrottle, 1,1 )


def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # node are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.sleep(4.0)
    print("SERIAL: Ignition Signal Sent")
    arduinomap.arduinoSerial(0,0,0,1)
    print("SERIAL: Start Signal Sent")
    rospy.sleep(2.0)
    arduinomap.arduinoSerial(0,0,1,1)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.Subscriber("cmd_vel", Twist, callback)
    
    rospy.spin()

if __name__ == '__main__':
    listener()
