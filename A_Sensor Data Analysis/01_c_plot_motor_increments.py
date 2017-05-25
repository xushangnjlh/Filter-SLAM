# Plot the increments of the left and right motor.
# 01_c_plot_motor_increments.py
# XU Shang, 2016-10-12
from pylab import plot,show
from lego_robot import LegoLogfile

if __name__ == '__main__':

    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    plot(logfile.motor_ticks)
    # motor_ticks is a tuple with motor ticks increment record from both
    # left and right wheel
    show()
