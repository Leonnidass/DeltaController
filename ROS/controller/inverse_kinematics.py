#!/usr/bin/env python

# delta_ik takes x,y and z as input and returns the actuator angles' values 
def delta_ik(x, y, z):
    import math

    e      = 26.73   # End effector equilateral triangle side length
    f      = 103.0   # Base equilateral triangle side length
    re     = 224.0   # Forearm length
    rf     = 98.4    # Bicep length

    sqrt3  = math.sqrt(3.0)
    pi     = math.pi
    sin120 = sqrt3 / 2.0
    cos120 = -0.5

    def inverse_kinematics_for_one_motor(x0, y0, z0):
        y1 = -0.5 * 0.57735 * f
        y0 -= 0.5 * 0.57735 * e
        a = (x0**2 + y0**2 + z0**2 + rf**2 - re**2 - y1**2) / (2.0 * z0)
        b = (y1 - y0) / z0
        d = -(a + b * y1)**2 + rf * (b**2 * rf + rf)
        if d < 0:
            return None  # non-existing point

        yj = (y1 - a * b - math.sqrt(d)) / (b**2 + 1)  
        zj = a + b * yj
        theta = math.atan(-zj / (y1 - yj)) * 180.0 / pi + ((yj > y1) * 180.0)
        return theta

    theta1 = inverse_kinematics_for_one_motor(x, y, z)
    theta2 = inverse_kinematics_for_one_motor(x * cos120 + y * sin120, y * cos120 - x * sin120, z)
    theta3 = inverse_kinematics_for_one_motor(x * cos120 - y * sin120, y * cos120 + x * sin120, z)

    return pi/180*theta1, pi/180*theta2, pi/180*theta3 # convert angles from degrees to radians

# Example usage:
# x, y, z = 0.0, 0.0, -300.0 
# angles = delta_ik_5(x, y, z)
# print(angles)

