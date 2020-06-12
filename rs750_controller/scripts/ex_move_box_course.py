#!/usr/bin/env python

''' Script to move robot around a box shaped course

    https://wiki.nps.edu/display/ROSSC/Assignment%3A+Piloting+a+Turtle
    https://wiki.nps.edu/pages/viewpage.action?pageId=904036402
'''

import math
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64

# Publishers
# Send commands to /cmd_vel_unscaled (geometry_msgs/Twist)
cmd_vel_pub = rospy.Publisher('/cmd_vel_unscaled', Twist, queue_size=10)

# Update rate
control_frequency = 10.0 # Hz

def move(speed):
    cmd_vel_msg = Twist()
    cmd_vel_msg.linear.x = speed
    cmd_vel_pub.publish(cmd_vel_msg)

def move_timed(speed, duration):
    start = rospy.get_rostime()
    rate = rospy.Rate(control_frequency)
    while rospy.get_rostime() - start < duration:
        move(speed)
        rate.sleep() 
    move(0.0)

def turn(yaw_rate):
    cmd_vel_msg = Twist()
    cmd_vel_msg.angular.z = yaw_rate
    cmd_vel_pub.publish(cmd_vel_msg)

def turn_timed(yaw_rate, duration):
    start = rospy.get_rostime()
    rate = rospy.Rate(control_frequency)
    while rospy.get_rostime() - start < duration:
        turn(yaw_rate)
        rate.sleep()
    turn(0.0)

if __name__ == '__main__':
    rospy.init_node('ex_move_box_course')
    rospy.loginfo('Starting node: ex_move_box_course')

    # Reset robot to origin


    # Parameters
    move_speed = 0.5
    move_duration = rospy.Duration.from_sec(3.0)
    turn_rate = 1.0
    turn_duration = rospy.Duration.from_sec(4.0)

    rospy.loginfo('move_speed: {}, move_duration: {} [s]'.format(move_speed, move_duration.to_sec()))
    rospy.loginfo('turn_rate:  {}, turn_duration: {} [s]'.format(turn_rate, turn_duration.to_sec()))

    # Leg 1
    rospy.loginfo('Leg 1')
    rospy.sleep(0.1)
    move_timed(move_speed, move_duration)

    # Leg 2
    rospy.loginfo('Leg 2')
    turn_timed(turn_rate, turn_duration)    
    move_timed(move_speed, move_duration)

    # Leg3
    rospy.loginfo('Leg 3')
    turn_timed(turn_rate, turn_duration)    
    move_timed(move_speed, move_duration)

    # Leg4
    rospy.loginfo('Leg 4')
    turn_timed(turn_rate, turn_duration)    
    move_timed(move_speed, move_duration)

    # Return to initial pose
    rospy.loginfo('Return to start pose')
    turn_timed(turn_rate, turn_duration)    

    rospy.loginfo('Done')
