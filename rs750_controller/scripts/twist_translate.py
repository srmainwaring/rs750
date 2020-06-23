#!/usr/bin/env python

# Copyright (C) 2020  Rhys Mainwaring
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

rudder_pub = rospy.Publisher('rudder_position/command', Float64, queue_size=10)
# main_sail_pub = rospy.Publisher('main_sail_position/command', Float64, queue_size=10)
# fore_sail_pub = rospy.Publisher('fore_sail_position/command', Float64, queue_size=10)

rudder_cmd_msg = Float64()
# main_sail_cmd_msg = Float64()
# fore_sail_cmd_msg = Float64()

def cmd_vel_cb(msg):
    # Translate twist message into position commands
    cmd1 = msg.linear.x * math.pi / 2
    cmd2 = msg.angular.z * math.pi / 2

    # Set commands 
    rudder_cmd_msg.data = cmd2
    # main_sail_cmd_msg.data = cmd1
    # fore_sail_cmd_msg.data = cmd1

    # Publish commands
    rudder_pub.publish(rudder_cmd_msg)
    # main_sail_pub.publish(main_sail_cmd_msg)
    # fore_sail_pub.publish(fore_sail_cmd_msg)

if __name__ == '__main__':
    rospy.init_node('twist_translate')
    cmv_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmd_vel_cb)
    rospy.spin()
