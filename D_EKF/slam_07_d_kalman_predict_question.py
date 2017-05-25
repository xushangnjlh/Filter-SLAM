# The complete Kalman prediction step (without the correction step).
#
# slam_07_d_kalman_predict_solution
# XU Shang, 2015-10-30
from lego_robot import *
from math import sin, cos, pi, atan2
from numpy import *


class ExtendedKalmanFilter:
    def __init__(self, state, covariance,
                 robot_width,
                 control_motion_factor, control_turn_factor):
        # The state. This is the core data of the Kalman filter.
        self.state = state
        self.covariance = covariance

        # Some constants.
        self.robot_width = robot_width
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

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
        """The prediction step of the Kalman filter."""
        # covariance' = G * covariance * GT + R
        # where R = V * (covariance in control space) * VT.
        # Covariance in control space depends on move distance.
        left, right = control

        # --->>> Put your code to compute the new self.covariance here.
        # First, construct the control_covariance, which is a diagonal matrix.
        # In Python/Numpy, you may use diag([a, b]) to get
        # [[ a, 0 ],
        #  [ 0, b ]].
        # Then, compute G using dg_dstate and V using dg_dcontrol.
        # Then, compute the new self.covariance.
        # Note that the transpose of a Numpy array G is expressed as G.T,
        # and the matrix product of A and B is written as dot(A, B).
        # Writing A*B instead will give you the element-wise product, which
        # is not intended here.
        sigma_control = diag([(self.control_motion_factor*left)**2+
                               (self.control_turn_factor*(left-right))**2,
                               (self.control_motion_factor*right)**2+
                               (self.control_turn_factor*(left-right))**2])
        G = self.dg_dstate(self.state,control,self.robot_width)
        V = self.dg_dcontrol(self.state,control,self.robot_width)
        
        R = dot(dot(V, sigma_control), V.T)
        self.covariance = dot(dot(G, self.covariance), G.T) + R
        self.state = self.g(self.state,control,self.robot_width)

        
        
        
        # state' = g(state, control)

        # --->>> Put your code to compute the new self.state here.


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Measured start position.
    initial_state = array([1850.0, 1897.0, 213.0 / 180.0 * pi])
    # Covariance at start position.
    initial_covariance = diag([100.0**2, 100.0**2, (10.0 / 180.0 * pi) ** 2])
    # Setup filter.
    kf = ExtendedKalmanFilter(initial_state, initial_covariance,
                              robot_width,
                              control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records and generate filtered positions and
    # covariances.
    # This is the Kalman filter loop, without the correction step.
    states = []
    covariances = []
    for m in logfile.motor_ticks:
        # Prediction.
        control = array(m) * ticks_to_mm
        kf.predict(control)

        # Log state and covariance.
        states.append(kf.state)
        covariances.append(kf.covariance)

    # Write all states, all state covariances, and matched cylinders to file.
    f = open("kalman_prediction.txt", "w")
    for i in xrange(len(states)):
        # Output the center of the scanner, not the center of the robot.
        print >> f, "F %f %f %f" % \
            tuple(states[i] + [scanner_displacement * cos(states[i][2]),
                               scanner_displacement * sin(states[i][2]),
                               0.0])
        # Convert covariance matrix to angle stddev1 stddev2 stddev-heading form
        e = ExtendedKalmanFilter.get_error_ellipse(covariances[i])
        print >> f, "E %f %f %f %f" % (e + (sqrt(covariances[i][2,2]),))

    f.close()
