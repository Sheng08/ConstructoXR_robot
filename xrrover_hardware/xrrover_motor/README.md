# xrrover_controller

## Launch Files
### 1. setup rosparameters
    ```bash
    roslaunch xrrover_controller loadRoverParam.launch
    # parameter for calculate each wheel's speed in mecanum_core.launch
    ```
    [ debug ] echo monitor 4 wheel's speed
    ```bash
    rostopic echo /motor/front/left
    rostopic echo /motor/front/right
    rostopic echo /motor/rear/left
    rostopic echo /motor/rear/right
    ```

### 2. calculate forward/inverse kinematics  
    ```bash
    roslaunch xrrover_controller mecanum_core.launch
    # take Twist convert to 4 wheel's speed, and capture wheel's real speed and publish odom
    ```

### 3. All-in-one(include 1. and 2.) launch file
    ```bash
    roslaunch xrrover_controller xrrover_mecanum.launch
    ```

## Basic Usage
### Concept  
    cmd_vel command ⟺ xrrover_mecanum ⟺ serial port
### Open Port for motor (require)
- manual
    ```bash
    sudo chmod 777 /dev/ttyUSB0
    rosrun rosserial_python serial_node.py /dev/ttyUSB0
    # exchange data between rosnode on arduino 
    ```
- [xrrover_connector](../xrrover_connector/READMD.md) launch file (prefer) 
    ```bash
    sudo chmod 777 /dev/ttyUSB0
    roslaunch xrrover_connector xrrover_motor.launch
    ```
### Control Command
- manually pub Twist msgs
    ```bash
    rostopic pub /cmd_vel geometry_msgs/Twist  '{linear:  {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0,y: 0.0,z: 0.0}}' --once
    ```
- [teleop_twist_keyboard](https://github.com/ros-teleop/teleop_twist_keyboard)
    ```bash
    rosrun teleop_twist_keyboard teleop_twist_keyboard.py
    ```
- [xrrover_navigation]() <!--# TODO: xrrover's nav package link -->
    ```bash
    roslaunch xrrover_navigation xrrover_navigation.launch
    ```
