# -*- coding: utf-8 -*-
"""
Created on Sat Oct 29 20:16:47 2015

@author: XU Shang

To calculate analytically the partial derivative of g (control model) with respect to state.
If the partial derivative with respect to x, y and theta (the state) are correct, 
then the numerical derivative and the analytical derivative should be the same.
"""
from lego_robot import LegoLogfile
from math import sin, cos, pi
from numpy import array, allclose, column_stack

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


if __name__ == '__main__':

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    l = 50.0
    r = 54.32
    control = array([l, r])
    w = 150.0

    # Compute derivative numerically.
    print "Numeric differentiation dx, dy, dtheta:"
    delta = 1e-7
    state_x = array([x + delta, y, theta])
    state_y = array([x, y + delta, theta])
    state_theta = array([x, y, theta + delta])
    dg_dx = (ExtendedKalmanFilter.g(state_x, control, w) -
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dy = (ExtendedKalmanFilter.g(state_y, control, w) -
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dtheta = (ExtendedKalmanFilter.g(state_theta, control, w) -
                 ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dstate_numeric = column_stack([dg_dx, dg_dy, dg_dtheta])
    print dg_dstate_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dx, dy, dtheta:"
    dg_dstate_analytic = ExtendedKalmanFilter.dg_dstate(state, control, w)
    print dg_dstate_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dstate_numeric - dg_dstate_analytic
    print "Seems correct:", allclose(dg_dstate_numeric, dg_dstate_analytic)

