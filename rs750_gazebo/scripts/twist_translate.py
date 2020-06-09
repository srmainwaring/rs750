#!/usr/bin/env python

import math
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

rudder_pub = rospy.Publisher('rudder_position/command', Float64, queue_size=10)
main_sail_pub = rospy.Publisher('main_sail_position/command', Float64, queue_size=10)
fore_sail_pub = rospy.Publisher('fore_sail_position/command', Float64, queue_size=10)

rudder_cmd_msg = Float64()
main_sail_cmd_msg = Float64()
fore_sail_cmd_msg = Float64()

def cmd_vel_cb(msg):
    # Translate twist message into position commands
    cmd1 = msg.linear.x * math.pi / 2
    cmd2 = msg.angular.z * math.pi / 2

    # Set commands 
    rudder_cmd_msg.data = cmd2
    main_sail_cmd_msg.data = cmd1
    fore_sail_cmd_msg.data = cmd1

    # Publish commands
    rudder_pub.publish(rudder_cmd_msg)
    main_sail_pub.publish(main_sail_cmd_msg)
    fore_sail_pub.publish(fore_sail_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('twist_translate')
    cmv_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
    rospy.spin()
