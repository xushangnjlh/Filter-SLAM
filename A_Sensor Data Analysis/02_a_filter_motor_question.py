# Implement the first move model for the Lego robot.
# 02_a_filter_motor
# XU Shang, 2016-10-12
from math import sin, cos, pi
from pylab import plot,show
from lego_robot import LegoLogfile

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width):

    # Find out if there is a turn at all.
    x0 = old_pose[0]
    y0 = old_pose[1]
    theta0 = old_pose[2]

    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        distance = motor_ticks[0]*ticks_to_mm
        x = x0 + distance*cos(theta0)
        y = y0 + distance*sin(theta0)
        theta = theta0
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R
        alpha = (motor_ticks[1] - motor_ticks[0])*ticks_to_mm/robot_width
        R = motor_ticks[0]*ticks_to_mm/alpha
        theta = (theta0+alpha)%(2*pi)
        x = x0 + (R+robot_width/2)*(sin(theta)-sin(theta0))
        y = y0 + (R+robot_width/2)*(-cos(theta)+cos(theta0))
        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 150.0

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Start at origin (0,0), Direct to x axis (alpha = 0).
    pose = (0.0, 0.0, 0.0)

    # Loop over all motor tick records to generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width)
        filtered.append(pose) 
        #pose is a tuple which can not be changed for every step

    # Draw result.
    for pose in filtered:
        print pose
        plot([p[0] for p in filtered], [p[1] for p in filtered], 'ro')
    show()
