# Implement the second move model for the Lego robot.
# The difference to the first implementation is:
# - added a scanner displacement
# - added a different start pose (measured in the real world)
# - result is now output to a file, as "F" ("filtered") records, which
#   can be recognized by logfile_viewer.py
#
# 02_b_filter_motor_file
# XU Shang, 2016-10-11
from math import sin, cos, pi
from lego_robot import LegoLogfile

# This function takes the old (x, y, heading) pose and the motor ticks
# (ticks_left, ticks_right) and returns the new (x, y, heading).
def filter_step(old_pose, motor_ticks, ticks_to_mm, robot_width,
                scanner_displacement):

    x0 = old_pose[0] 
    y0 = old_pose[1]
    theta0 = old_pose[2]
    # Find out if there is a turn at all.
    if motor_ticks[0] == motor_ticks[1]:
        # No turn. Just drive straight.
        distance = motor_ticks[0]*ticks_to_mm
        x = x0 + distance*cos(theta0)
        y = y0 + distance*sin(theta0)
        theta = theta0
        
        return (x, y, theta)

    else:
        # Turn. Compute alpha, R, etc.

        # --->>> Modify your previous implementation.
        # First modify the the old pose to get the center (because the
        #   old pose is the LiDAR's pose, not the robot's center pose).
        xc = x0 - scanner_displacement*cos(theta0)
        yc = y0 - scanner_displacement*sin(theta0)
        thetac = theta0
        # Second, execute your old code, which implements the motion model
        #   for the center of the robot.
        alpha = (motor_ticks[1] - motor_ticks[0])*ticks_to_mm/robot_width
        R = motor_ticks[0]*ticks_to_mm/alpha
        thetac2 = (thetac+alpha)%(2*pi)
        xc2 = xc + (R+robot_width/2)*(sin(thetac2)-sin(thetac))
        yc2 = yc + (R+robot_width/2)*(-cos(thetac2)+cos(thetac))
        # Third, modify the result to get back the LiDAR pose from
        #   your computed center. This is the value you have to return.
        theta = thetac2
        x = xc2 + scanner_displacement*cos(theta)
        y = yc2 + scanner_displacement*sin(theta)

        return (x, y, theta)

if __name__ == '__main__':
    # Empirically derived distance between scanner and assumed
    # center of robot.
    scanner_displacement = 30.0

    # Empirically derived conversion from ticks to mm.
    ticks_to_mm = 0.349

    # Measured width of the robot (wheel gauge), in mm.
    robot_width = 172.0

    # Measured start position of LiDRA
    pose = (1850.0, 1897.0, 213.0 / 180.0 * pi)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records generate filtered position list.
    filtered = []
    for ticks in logfile.motor_ticks:
        pose = filter_step(pose, ticks, ticks_to_mm, robot_width,
                           scanner_displacement)
        filtered.append(pose)

    # Write all filtered positions to file.
    f = open("poses_from_ticks.txt", "w")
    for pose in filtered:
        print >> f, "F %f %f %f" % pose
    f.close()
