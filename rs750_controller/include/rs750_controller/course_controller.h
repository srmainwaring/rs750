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

#ifndef RS750_CONTROLLER_COURSE_CONTROLLER_H
#define RS750_CONTROLLER_COURSE_CONTROLLER_H

#include <ros/ros.h>
#include <control_toolbox/pid.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#include <string>
#include <vector>

// Guide
// https://github.com/awesomebytes/control_toolbox_pid_tutorial
// http://docs.ros.org/api/control_toolbox/html/classcontrol__toolbox_1_1Pid.html
//
// Objective
//
//  Keep A robot on a constant heading (fixed yaw value)
//
//

namespace rs750_controller
{
    class CourseController
    {
    public:
        virtual ~CourseController();

        CourseController(ros::NodeHandle nh, ros::NodeHandle private_nh);

        void read(const ros::Time &time);
        void update(const ros::Time &time);
        void write(const ros::Time &time);

    private:
        void speed_cb(const std_msgs::Float64ConstPtr &msg);
        void yaw_cb(const std_msgs::Float64ConstPtr &msg);
        void rpy_cb(const geometry_msgs::Vector3StampedConstPtr &msg);

        // ROS node handle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // Pid
        ros::Time last_time_;
        control_toolbox::Pid pid_;

        // Subscribers
        double speed_;
        const std::string speed_topic_ = "speed";
        ros::Subscriber speed_sub_;

        double yaw_sp_;
        const std::string yaw_topic_ = "yaw";
        ros::Subscriber yaw_sub_;

        double yaw_pv_;
        const std::string rpy_topic_ = "rpy";
        ros::Subscriber rpy_sub_;

        // Publishers
        double command_;
        const std::string cmd_vel_topic_ = "cmd_vel";
        geometry_msgs::Twist cmd_vel_msg_;
        ros::Publisher cmd_vel_pub_;

    };

} // namespace rs750_controller

#endif // RS750_CONTROLLER_COURSE_CONTROLLER_H
