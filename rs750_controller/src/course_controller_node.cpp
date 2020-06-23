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

#include "rs750_controller/course_controller.h"

int main(int argc, char* argv[])
{
    // Initialise node
    ros::init(argc, argv, "course_controller");
    ros::NodeHandle nh, private_nh("~");

    // Parameters
    double control_frequency = 10.0;

    // Controller
    auto controller = rs750_controller::CourseController(nh, private_nh); 

    // Loop
    ros::Rate rate(control_frequency);
    while (ros::ok())
    {
        ros::Time now = ros::Time::now();

        controller.read(now);
        controller.update(now);
        controller.write(now);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}