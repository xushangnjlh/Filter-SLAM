# -*- coding: utf-8 -*-
"""
Created on Wed Nov 09 21:42:57 2016

@author: Administrator
"""
from lego_robot import LegoLogfile
from pylab import *

if __name__ == '__main__':
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
                
    plot(logfile.motor_ticks)
    show()