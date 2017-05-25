# This is the EKF implementation, consisting of prediction (motion) and correction (perception)
# The filtered trajectory and covariance will be output to a txt file,
# which can by parsed by and visualized by logfile_viewer.py  
# Also, raw data from LiDAR sensors should be pre-processed
#
# XU Shang, 2015-10-30

from lego_robot import *
from math import sin, cos, pi, atan2, sqrt
from numpy import *
from slam_d_library import get_observations, write_cylinders
class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor,
                 measurement_distance_stddev, measurement_angle_stddev):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor
        self.measurement_distance_stddev = measurement_distance_stddev
        self.measurement_angle_stddev = measurement_angle_stddev

    @staticmethod
    def g(state, control, w):
        x, y, theta = state
        l, r = control
        if r != l:
            alpha = (r - l) / w
            rad = l/alpha
            g1 = x + (rad + w/2.)*(sin(theta+alpha) - sin(theta))
            g2 = y + (rad + w/2.)*(-cos(theta+alpha) + cos(theta))
            g3 = (theta + alpha + pi) % (2*pi) - pi
        else:
            g1 = x + l * cos(theta)
            g2 = y + l * sin(theta)
            g3 = theta
        return array([g1, g2, g3])

    @staticmethod
    def dg_dstate(state, control, w):
        theta = state[2]
        l, r = control
        if r != l:
            alpha = (r-l)/w
            R = l/alpha
            g1_x = 1
            g1_y = 0
            g1_theta = (R + w/2.0) * (cos(theta+alpha) - cos(theta))
            g2_x = 0
            g2_y = 1
            g2_theta = (R + w/2.0) * (sin(theta+alpha) - sin(theta))
            g3_x = 0
            g3_y = 0
            g3_theta = 1
        else:
            g1_x = 1
            g1_y = 0
            g1_theta = -l*sin(theta)
            g2_x = 0 
            g2_y = 1
            g2_theta = l*cos(theta)
            g3_x = 0
            g3_y = 0
            g3_theta = 1
        m = array([[g1_x, g1_y, g1_theta],
                   [g2_x, g2_y, g2_theta], 
                   [g3_x, g3_y, g3_theta]])
        return m

    @staticmethod
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = tuple(control)
        if r != l:
            alpha = (r-l)/w
            g1_l = w*r/((r-l)*(r-l)) * (sin(theta+alpha)-sin(theta)) - (r+l)/(2*(r-l))*cos(theta+alpha)
            g2_l = w*r/((r-l)*(r-l)) * (-cos(theta+alpha)+cos(theta)) - (r+l)/(2*(r-l))*sin(theta+alpha)
            g3_l = -1/w
            g1_r = -w*l/((r-l)*(r-l)) * (sin(theta+alpha)-sin(theta)) + (r+l)/(2*(r-l))*cos(theta+alpha)
            g2_r = -w*l/((r-l)*(r-l)) * (-cos(theta+alpha)+cos(theta)) + (r+l)/(2*(r-l))*sin(theta+alpha)
            g3_r = 1/w
        else:
            g1_l = 1/2.0 * (cos(theta) + l/w*sin(theta))
            g2_l = 1/2.0 * (sin(theta) - l/w*cos(theta))
            g3_l = -1/w
            g1_r = 1/2.0 * (-l/w*sin(theta) + cos(theta))
            g2_r = 1/2.0 * (l/w*cos(theta) + sin(theta))
            g3_r = 1/w
        m = array([[g1_l, g1_r], [g2_l, g2_r], [g3_l, g3_r]]) 
        return m

    @staticmethod
    def get_error_ellipse(covariance):
        """Return the position covariance (which is the upper 2x2 submatrix)
           as a triple: (main_axis_angle, stddev_1, stddev_2), where
           main_axis_angle is the angle (pointing direction) of the main axis,
           along which the standard deviation is stddev_1, and stddev_2 is the
           standard deviation along the other (orthogonal) axis."""
        eigenvals, eigenvects = linalg.eig(covariance[0:2,0:2])
        angle = atan2(eigenvects[1,0], eigenvects[0,0])
        return (angle, sqrt(eigenvals[0]), sqrt(eigenvals[1]))        

    def predict(self, control):
        left, right = control
        sigma_control = diag([(self.control_motion_factor*left)**2+
                              (self.control_turn_factor*(left-right))**2,
                              (self.control_motion_factor*right)**2+
                              (self.control_turn_factor*(left-right))**2])
        G = self.dg_dstate(self.state,control,self.robot_width)
        V = self.dg_dcontrol(self.state,control,self.robot_width)
        R = dot(dot(V, sigma_control), V.T)
        self.covariance = dot(dot(G, self.covariance), G.T) + R
        self.state = self.g(self.state,control,self.robot_width)

    @staticmethod
    def h(state, landmark, scanner_displacement):
        """Takes a (x, y, theta) state and a (x, y) landmark, and returns the
           measurement (range, bearing)."""
        dx = landmark[0] - (state[0] + scanner_displacement * cos(state[2]))
        dy = landmark[1] - (state[1] + scanner_displacement * sin(state[2]))
        r = sqrt(dx * dx + dy * dy)
        alpha = (atan2(dy, dx) - state[2] + pi) % (2*pi) - pi
        return array([r, alpha])

    @staticmethod
    def dh_dstate(state, landmark, scanner_displacement):
        xl = state[0]
        yl = state[1]
        theta = state[2]
        xm = landmark[0]
        ym = landmark[1]
        r2 = (xm-xl)**2 + (ym-yl)**2
        delta_x = xm - xl
        delta_y = ym - yl
        # r is the range, b is the bearing
        dr_x = -delta_x/sqrt(r2)
        dr_y = -delta_y/sqrt(r2)
        dr_theta = scanner_displacement / sqrt(r2) * (delta_x*sin(theta) - delta_y*cos(theta))
        db_x = delta_y/r2
        db_y = -delta_x/r2
        db_theta =  -scanner_displacement / r2 * (delta_x*cos(theta) + delta_y*sin(theta)) -1
        return array([[dr_x, dr_y, dr_theta], [db_x, db_y, db_theta]])

    def correct(self, measurement, landmark):
        """The correction step of the Kalman filter."""
        H = self.dh_dstate(self.state,landmark,self.scanner_displacement)
        # Q, a diagonal matrix, from self.measurement_distance_stddev and
        #  self.measurement_angle_stddev (remember: Q contains variances).
        Q = diag([self.measurement_distance_stddev**2, self.measurement_angle_stddev**2])
        # K, from self.covariance, H, and Q.
        K = dot(dot(self.covariance, H.T),linalg.inv(dot(dot(H, self.covariance), H.T) + Q))
        #  Use linalg.inv(...) to compute the inverse of a matrix.
        # The innovation: it is easy to make an error here, because the
        #  predicted measurement and the actual measurement of theta may have
        #  an offset of +/- 2 pi. So here is a suggestion:
        #   innovation = array(measurement) -\
        #                self.h(self.state, landmark, self.scanner_displacement)
        #   innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        innovation = array(measurement) - self.h(self.state ,landmark, self.scanner_displacement)
        innovation[1] = (innovation[1] + pi) % (2*pi) - pi
        # Compute the new self.state.
        self.state = self.state + dot(K, innovation)
        # And finally, compute the new self.covariance. Use eye(3) to get a 3x3
        #  identity matrix.
        self.covariance = dot((eye(3) - dot(K, H)), self.covariance)
        #
        # Hints:
        # dot(A, B) is the 'normal' matrix product (do not use: A*B).
        # A.T is the transposed of a matrix A (A itself is not modified).
        # linalg.inv(A) returns the inverse of A (A itself is not modified).
        # eye(3) returns a 3x3 identity matrix.

if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 153.0

    # Cylinder extraction and matching constants.
    minimum_valid_distance = 20.0
    depth_jump = 100.0
    cylinder_offset = 90.0
    max_cylinder_distance = 300.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.
    measurement_distance_stddev = 200.0  # Distance measurement error of cylinders.
    measurement_angle_stddev = 15.0 / 180.0 * pi  # Angle measurement error.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width, scanner_displacement,
                              control_motion_factor, control_turn_factor,
                              measurement_distance_stddev,
                              measurement_angle_stddev)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")
    logfile.read("robot4_scan.txt")
    logfile.read("robot_arena_landmarks.txt")
    reference_cylinders = [l[1:3] for l in logfile.landmarks]

    # Loop over all motor tick records and all measurements and generate
    # filtered positions and covariances.
    # This is the Kalman filter loop, with prediction and correction.
    states = []
    covariances = []
    matched_ref_cylinders = []
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = array(logfile.motor_ticks[i]) * ticks_to_mm
        kf.predict(control)

        # Correction.
        observations = get_observations(
            logfile.scan_data[i],
            depth_jump, minimum_valid_distance, cylinder_offset,
            kf.state, scanner_displacement,
            reference_cylinders, max_cylinder_distance)
        for j in xrange(len(observations)):
            kf.correct(*observations[j])

        # Log state, covariance, and matched cylinders for later output.
        states.append(kf.state)
        covariances.append(kf.covariance)
        matched_ref_cylinders.append([m[1] for m in observations])

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction_and_correction.txt", "w")
    for i in xrange(len(states)):
        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                               scanner_displacement * sin(states[i][2]),
                               0.0])
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))
        # Also, write matched cylinders.
        write_cylinders(f, "W C", matched_ref_cylinders[i])        

    f.close()
