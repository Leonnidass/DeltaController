This folder will contain the simulation of the Delta Robot and the Joystick Controller.

![rsz_screenshot_from_2024-01-27_15-46-41](https://github.com/Leonnidass/DeltaController/assets/100625531/08c2969f-123a-491f-8c14-a1adfab7af66)



# Installing the Joystick Drivers
```bash
sudo apt-get install ros-noetic-joy
sudo apt-get install ros-noetic-joystick-drivers
```
# Creating a workspace
```bash
mkdir delta_ws/src
cd delta_ws/src
git clone https://github.com/Leonnidass/DeltaController/tree/main/ROS.git
cd delta_ws
catkin_make
```
# Launchig the simulation
```bash
roslaunch description load_robot.launch
```

# Launching the controller
```bash
roslaunch controller joy_controller.launch
```
