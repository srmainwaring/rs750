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

#ifndef RS750_CONTROLLER_HARDWARE_H
#define RS750_CONTROLLER_HARDWARE_H

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace rs750_controller
{
    class Hardware : public hardware_interface::RobotHW
    {
    public:
        virtual ~Hardware();

        Hardware(ros::NodeHandle nh, ros::NodeHandle private_nh);

        void read(const ros::Time &time, const ros::Duration &period) override;

        void write(const ros::Time &time, const ros::Duration &period) override;

    private:
        // ROS node handle
        ros::NodeHandle nh_;
        ros::NodeHandle private_nh_;

        // ros_control interfaces: joints, position for rudder and sail trim.
        hardware_interface::JointStateInterface joint_state_interface_;
        hardware_interface::VelocityJointInterface velocity_joint_interface_;
        hardware_interface::PositionJointInterface position_joint_interface_;

        // ROS parameters
        
        // Rudder joint - position controlled
        std::vector<double> rudder_joint_positions_;
        std::vector<double> rudder_joint_velocities_;
        std::vector<double> rudder_joint_efforts_;
        std::vector<double> rudder_joint_velocity_commands_;
        std::vector<std::string> rudder_joint_names_;

        // Sail joints - position limit controlled
        std::vector<double> sail_joint_positions_;
        std::vector<double> sail_joint_velocities_;
        std::vector<double> sail_joint_efforts_;
        std::vector<double> sail_joint_position_commands_;
        std::vector<std::string> sail_joint_names_;

        void registerControlInterfaces();
    };

} // namespace rs750_controller

#endif // RS750_CONTROLLER_HARDWARE_H
