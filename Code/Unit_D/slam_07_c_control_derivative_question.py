# This adds the derivative of g, this time with respect to the control
# (left and right motor movement).
#
# slam_07_c_control_derivative
# Claus Brenner, 11.12.2012
from lego_robot import *
from math import sin, cos, pi
from numpy import *

class ExtendedKalmanFilter:

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
    def dg_dcontrol(state, control, w):
        theta = state[2]
        l, r = control
        alpha = (r - l) / w
        theta_prime = theta+alpha
        r_plus_l = r+l
        r_minus_l = r-l
        l_divide_w = l/w
        if r != l:
           
            g1_dl = w*r/(r_minus_l*r_minus_l) * (sin(theta_prime)-sin(theta)) - r_plus_l/(2*r_minus_l) * cos(theta_prime)
            g2_dl = w*r/(r_minus_l*r_minus_l) * (-cos(theta_prime)+cos(theta)) - r_plus_l/(2*r_minus_l) * sin(theta_prime)
            g3_dl = -1/w
            
            g1_dr = -(w*l)/(r_minus_l*r_minus_l) * (sin(theta_prime)-sin(theta)) + r_plus_l/(2*r_minus_l) * cos(theta_prime)
            g2_dr = -(w*l)/(r_minus_l*r_minus_l) * (-cos(theta_prime)+cos(theta)) + r_plus_l/(2*r_minus_l)  *sin(theta_prime)
            g3_dr = 1/w
            m = array([[g1_dl,g1_dr],[g2_dl,g2_dr],[g3_dl,g3_dr]])  # Replace this.

        else:
            g1_dl = 0.5 * (cos(theta) +  l_divide_w* sin(theta))
            g2_dl = 0.5 * (sin(theta) - l_divide_w * cos(theta))
            g3_dl = -1/w
            
            g1_dr = 0.5 * (-l_divide_w * sin(theta) + cos(theta))
            g2_dr = 0.5 * (l_divide_w * cos(theta) + sin(theta))
            g3_dr = 1/w
            # --->>> Put your code here.
            # This is for the special case r == l.
            m = array([[g1_dl,g1_dr],[g2_dl,g2_dr],[g3_dl,g3_dr]]) # Replace this.

            
        return m


if __name__ == '__main__':
    # If the partial derivative with respect to l and r (the control)
    # are correct, then the numerical derivative and the analytical
    # derivative should be the same.

    # Set some variables. Try other variables as well.
    # In particular, you should check cases with l == r and l != r.
    x = 10.0
    y = 20.0
    theta = 35. / 180. * pi
    state = array([x, y, theta])
    l = 40.0
    r = 40.0
    control = array([l, r])
    w = 150.0

    # Compute derivative numerically.
    print "Numeric differentiation dl, dr"
    delta = 1e-7
    control_l = array([l + delta, r])
    control_r = array([l, r + delta])
    dg_dl = (ExtendedKalmanFilter.g(state, control_l, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dr = (ExtendedKalmanFilter.g(state, control_r, w) -\
             ExtendedKalmanFilter.g(state, control, w)) / delta
    dg_dcontrol_numeric = column_stack([dg_dl, dg_dr])
    print dg_dcontrol_numeric

    # Use the above code to compute the derivative analytically.
    print "Analytic differentiation dl, dr:"
    dg_dcontrol_analytic = ExtendedKalmanFilter.dg_dcontrol(state, control, w)
    print dg_dcontrol_analytic

    # The difference should be close to zero (depending on the setting of
    # delta, above).
    print "Difference:"
    print dg_dcontrol_numeric - dg_dcontrol_analytic
    print "Seems correct:", allclose(dg_dcontrol_numeric, dg_dcontrol_analytic)
