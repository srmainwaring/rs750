# RS750 Robot Model

A robot model for designer Bryn Heveldt's Racing Sparrow 750 yacht
[http://www.racingsparrow.co.nz/](http://www.racingsparrow.co.nz/).

## Overview

These packages contain a robot model based upon the Racing Sparrow 750
model yacht. They include a robot model that may be viewed in `rviz` and 
simulated in Gazebo using the plugins from the `asv_wave_sim` project. 

The project is intended to be used as a test bed for developing
and prototyping ROS packages for robotic sailing vessels. 

## Usage

Update the Gazebo plugin path:

```bash
# Source the Gazebo environment (add this to ~/.bash_profile)
source /usr/local/share/gazebo/setup.sh

# Add the workspace libraries to the plugin path
export GAZEBO_PLUGIN_PATH=${PWD}/devel/lib:$GAZEBO_PLUGIN_PATH

# Check the Gazebo environment variables
printenv |grep GAZEBO
```

Launch the ocean world and spawn the model:

```bash
roslaunch rs750_gazebo rs750_ocean_world.launch verbose:=true
```

The `verbose:=true` flag provides additional information including whether
the plugins are found and load correctly.

## License

This is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This software is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
[GNU General Public License](LICENSE) for more details.

## Acknowledgments

The Racing Sparrow 750 was designed by Bryn Heveldt and is used here with his kind permission.
The model plans and original 3D CAD files are available at:

- [Racing Sparrow Model RC Yachts](http://www.racingsparrow.co.nz/theboat/)

