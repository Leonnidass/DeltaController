#!/usr/bin/env python3

from inverse_kinematics import delta_ik
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Joy
import rospy
import numpy as np
import time 

angles  = Float64MultiArray()
vel     = 2
x_pos   = 0
y_pos   = 0
z_pos   = -200
max_vel = 10
min_vel = 0.5
acc     = 0.1

joystick_state = {
                  'x': 0,
                  'y': 0, 
                  'z': 0,
                  'reset': False,
                  'circle': False,
                  'speed_up': False,
                  'speed_down': False
                }

def joyCallback(data):
    global joystick_state
    joystick_state['x']          = round(data.axes[0], 4)
    joystick_state['y']          = round(data.axes[1], 4)
    joystick_state['z']          = round(data.axes[4], 4)
    joystick_state['reset']      = data.buttons[0]
    joystick_state['circle']     = data.buttons[1]
    joystick_state['speed_up']   = data.buttons[13]
    joystick_state['speed_down'] = data.buttons[14]

def circle():
    z_pos = -200
    for i in range(100):
        x_pos = 50*np.cos(-2*np.pi*i/100)
        y_pos = 50*np.sin(-2*np.pi*i/100)
        theta1, theta2, theta3 = delta_ik(x_pos, y_pos, z_pos)
        angles.data = [theta1, theta2, theta3]
        pub.publish(angles)
        time.sleep(0.05) 

def send_command(event):
    global x_pos, y_pos, z_pos, angles, vel
    if joystick_state['reset']:
        x_pos, y_pos, z_pos = 0, 0, -200
    elif joystick_state['speed_up']:
        if vel<max_vel:
            vel+= acc
    elif joystick_state['speed_down']:
        if vel>min_vel:
            vel-= acc   
    elif joystick_state['circle']:
        circle()
    else:
        x_pos += joystick_state['x'] * vel
        y_pos += joystick_state['y'] * vel
        z_pos += joystick_state['z'] * vel

    theta1, theta2, theta3 = delta_ik(x_pos, y_pos, z_pos)
    angles.data = [theta1, theta2, theta3]

    
    pub.publish(angles)

    print('theta1 = {}\ntheta2 = {}\ntheta3 = {}\nvelocity = {}'.format(round(theta1,4),
                                                                        round(theta2,4),
                                                                        round(theta3,4),
                                                                        round(vel,4)))
    print('--------------------------------')

def listener():
    rospy.init_node('joy_controller')
    rospy.Subscriber("joy", Joy, joyCallback)
    rospy.Timer(rospy.Duration(0.05), send_command)
    print('Starting Joy Controller')
    rospy.spin() 

pub = rospy.Publisher('/delta_robot_position_controller/command', Float64MultiArray, queue_size=10)

if __name__ == '__main__':
    listener()
