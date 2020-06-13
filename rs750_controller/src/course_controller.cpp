//
//  Software License Agreement (BSD-3-Clause)
//   
//  Copyright (c) 2020 Rhys Mainwaring
//  All rights reserved
//   
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//  1.  Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//  2.  Redistributions in binary form must reproduce the above
//      copyright notice, this list of conditions and the following
//      disclaimer in the documentation and/or other materials provided
//      with the distribution.
//
//  3.  Neither the name of the copyright holder nor the names of its
//      contributors may be used to endorse or promote products derived
//      from this software without specific prior written permission.
// 
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.
//


#include "rs750_controller/course_controller.h"
#include <cmath>

namespace rs750_controller
{
    /// \brief Ensure the angle is in [0, 2 pi]
    double standardiseAngle(double a)
    {
        if (a < 0.0)
        {
            a += 2 * M_PI;
        }
        return a;
    }

    /// \brief Calculate the difference between two angles.
    double angleDiff(double a, double b)
    {
        // Map both angles to [0, 2 * pi]
        a = standardiseAngle(a);
        b = standardiseAngle(b);
        double d = a - b;

        // Map difference to [-pi, pi]
        if (d > M_PI)
        {
            return 2 * M_PI - d;
        }
        if (d < - M_PI)
        {
            return  2 * M_PI + d;
        }
        return d;
    }

    CourseController::~CourseController()
    {
    }

    CourseController::CourseController(ros::NodeHandle nh, ros::NodeHandle private_nh) :
        nh_(nh),
        private_nh_(private_nh)
    {
        // Subscribers
        speed_sub_ = nh.subscribe(speed_topic_, 10, &CourseController::speed_cb, this);
        yaw_sub_ = nh.subscribe(yaw_topic_, 10, &CourseController::yaw_cb, this);
        rpy_sub_ = nh.subscribe(rpy_topic_, 10, &CourseController::rpy_cb, this);

        // Publishers
        cmd_vel_pub_ = nh.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 10);

        // PID controller       
        ros::Time last_time_ = ros::Time::now();
        pid_.init(private_nh);        
    }

    void CourseController::read(const ros::Time &time)
    {
        // No-op - reading is done in subscriptions (not RT safe)
    }

    void CourseController::update(const ros::Time &time)
    {
        // yaw_sp_ is target,  yaw_pv_ is state, the difference is between two angles.
        double error = angleDiff(yaw_sp_, yaw_pv_);
        ros::Duration dt = time - last_time_;
        command_ = pid_.computeCommand(error, dt);
        last_time_ = time;
        ROS_DEBUG_STREAM("yaw_sp: " << yaw_sp_ << ", yaw_pv: " << yaw_pv_ << ", err: " << error);
    }

    void CourseController::write(const ros::Time &time)
    {
        // Set speed and yaw rate to /cmd_vel
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = speed_;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = 0.0;
        cmd_vel.angular.x = 0.0;
        cmd_vel.angular.y = 0.0;
        cmd_vel.angular.z = command_;
        cmd_vel_pub_.publish(cmd_vel);
    }

    void CourseController::speed_cb(const std_msgs::Float64ConstPtr &msg)
    {
        speed_ = msg->data;
    }

    void CourseController::yaw_cb(const std_msgs::Float64ConstPtr &msg)
    {
        // The set point is the desired yaw
        yaw_sp_ = msg->data;
    }

    void CourseController::rpy_cb(const geometry_msgs::Vector3StampedConstPtr &msg)
    {
        // The process variable is the measured yaw
        yaw_pv_ = msg->vector.z;
    }


} // namespace rs750_controller
