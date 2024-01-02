# ![dar](https://github.com/Leonnidass/DeltaController/assets/100625531/3fc2268a-4a76-40a5-91ed-d7259847ed39)


This repository will contain all my internship work on the X-Delta Robot.
- The controllers (OpenCR controller and ROS U2D2 controllers)
- The kinematic representation.
- The 3D SolidWorks modeling
- The URDF Visualisation
- The simulations
- The PID Tuning. 

# Controlling the robot
I will start by presenting the OpenCR Controller
- This is a full and easy-to-use controller for an X-Delta Robot with 3 dynamixel servos, using an OpenCR board
- I will explain the process step by step so everyone can implement the controller and use it.
  

![Capture d'écran 2023-12-05 165753s](https://github.com/Leonnidass/DeltaController/assets/100625531/937f559a-fdde-446f-bb32-ba2e67ba59bb)

## The hardware connection
- The OpenCR1.0 is a development board created by ROBOTIS for ROS systems to provide completely open-source hardware and software.
- The development environment for OpenCR1.0 is Arduino IDE.

![x](https://github.com/Leonnidass/DeltaController/assets/100625531/ab61c5de-8578-46ca-9773-1befaa4145bf)

- Our Robot uses 3 XL430-W250-T Servos, that support Daisy-Chain Communication; we control each DYNAMIXEL by sending data packets through a single shared communication bus, and every DYNAMIXEL can be connected to another DYNAMIXEL thus only one cable connection to the controller and power is required.

![opencrdelta](https://github.com/Leonnidass/DeltaController/assets/100625531/6c9f236d-b982-4148-bedc-5701679f116b)

## Setting up the IDE
- After installing the Arduino IDE, we need to install the OpenCR Library so that we can program it.
- File > Preferences > Additional Boards Manager URLs
- Copy this link
```bash
https://raw.githubusercontent.com/ROBOTIS-GIT/OpenCR/master/arduino/opencr_release/package_opencr_index.json
```
- We can now install the OpenCR Boad.

## Installing the libraries
- To set up the board, we need to install the OpenCR Library, which contains the 2 libraries that we need: OpenManipulator library and RobotisManipulator library.

## Setting up the Board
- Open the open_manipulator_delta folder
- Upload the open_manipulator_delta.ino file to the OpenCR
