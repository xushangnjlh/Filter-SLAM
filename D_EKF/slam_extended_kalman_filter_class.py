# -*- coding: utf-8 -*-
"""
Created on Sat Oct 29 18:59:07 2015

@author: XU Shang

This is a comparison demo to show without EKF, 
the control model will deviate far from the reference postion 
"""

from lego_robot import LegoLogfile
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:
    
    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if l!=r:
            alpha = (r-l)/w
            R = l/alpha
            g1 = x + (R + w/2.0)*(sin(theta+alpha) - sin(theta))
            g2 = y + (R + w/2.0)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha) % (2*pi)
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta
            
        return array([g1, g2, g3])

if __name__ == '__main__':
    scanner_displacement = 30.0 # scanner's position related to robot's coordinate
    ticks_to_mm = 0.349 # convert ticks from motor to distance
    robot_width = 155.0 
    # initial state
    state = array([1850.0, 1897.0, 213.0/180*pi])
    # read data from motor's tick sensor
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    
    states = []

    # loop over all motor ticks to generate a states list
    for ticks in logfile.motor_ticks:
        control = array(ticks)*ticks_to_mm
        state = ExtendedKalmanFilter.g(state, control, robot_width)
        states.append(state)
        
    f = open("states_from_ticks.txt","w")
    # output the state of scanner instead of the robot
    for s in states:
        print >> f, "F %f %f %f" % \
            tuple(s + [scanner_displacement*cos(s[2]), scanner_displacement*sin(s[2]),0.0])
    f.close()