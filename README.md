# RS750 Robot Model

A robot model for a Racing Sparrow 750 yacht.

## Overview

These packages contain a robot model based upon the Racing Sparrow 750
model yacht. They include a robot model that may be viewed in `rviz` and 
simulated in Gazebo using the plugins from the `asv_wave_sim` project. 

The project is intended to be used as a test bed for developing
and prototyping ROS packages for robotic sailing vessels. 

## Usage

Update the Gazebo plugin path:

```bash
# Source the Gazebo environment (add this to ~/.zprofile)
source /usr/local/share/gazebo-11/setup.sh

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

BSD 3 Clause

## Acknowledgments

