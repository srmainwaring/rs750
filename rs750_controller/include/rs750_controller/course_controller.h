// Copyright (C) 2020  Rhys Mainwaring
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

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
