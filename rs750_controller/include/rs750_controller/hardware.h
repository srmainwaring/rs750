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
        hardware_interface::PositionJointInterface position_joint_interface_;

        // ROS parameters
        
        // Rudder joint - position controlled
        std::vector<double> rudder_joint_positions_;
        std::vector<double> rudder_joint_velocities_;
        std::vector<double> rudder_joint_efforts_;
        std::vector<double> rudder_joint_position_commands_;
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
