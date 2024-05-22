# ConstructoXR_robot

## Add demo_map_model for Gazebo
```bash=
mkdir -p ~/.gazebo/models

cp -r demo_map_model ~/.gazebo/models/demo
```

## Setup ROS environment
> Architecture: x86
> OS: Linux (Ubuntu 20.04.6 LTS)
> ROS version: **noetic**

### If your system does not have ROS (Optional)
```base=
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update

sudo apt install ros-noetic-desktop-full

echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc

source ~/.bashrc

source /opt/ros/noetic/setup.bash

# then open new terminal(shell)
```

#### Install Dependency
```bash=
sudo apt install python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential

sudo rosdep init

rosdep update
```

#### Verify ROS
```
roscore
```


## Build this project
> Use catkin_make

### Install necessary packages
```bash=
sudo apt install \
    ros-noetic-amcl \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-gazebo-ros-control
``` 

### Building
```bash=
mkdir src

mv xrrover xrrover_function xrrover_hardware xrrover_unity src/

catkin_make

source devel/setup.bash
```

## Run XRrover Navigate Simulation

## Make sure roscore is runing
```bash
roscore
```

## launch order
```bash=
roslaunch xrrover_simulation xrrover_sim.launch

roslaunch xrrover_navigation xrrover_navigation.launch

roslaunch xrrover_bringup xrrover_model.launch
```

or run all in one

```bash=
load_all_in_one.launch
```

---

## Helpful tools
```
rosrun rqt_reconfigure rqt_reconfigure

rosrun tf view_frames

rosrun tf tf_echo odom map

rosrun tf tf_monitor

rostopic hz /scan

rostopic echo

rostopic echo /odom

rosparam get /use_sim_time
```