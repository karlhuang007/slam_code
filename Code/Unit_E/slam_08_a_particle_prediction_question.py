# The particle filter, prediciton only.
#
# slam_08_a_particle_prediciton.
# Claus Brenner, 04.01.2013
from lego_robot import *
from math import sin, cos, pi, sqrt
import random

class ParticleFilter:
    def __init__(self, initial_particles,
                 robot_width, scanner_displacement,
                 control_motion_factor, control_turn_factor):
        # The particles.
        self.particles = initial_particles

        # Some constants.
        self.robot_width = robot_width
        self.scanner_displacement = scanner_displacement
        self.control_motion_factor = control_motion_factor
        self.control_turn_factor = control_turn_factor

    # State transition. This is exactly the same method as in the Kalman filter.
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

        return (g1, g2, g3)

    def predict(self, control):
        """The prediction step of the particle filter."""
        l, r = control
        #left_center = sum(l)/len(control)
        #right_center = sum(r)/len(control)
        alpha1 = self.control_motion_factor
        alpha2 = self.control_turn_factor
        # --->>> Put your code here.
        # calculate the standard diviation of Normal distrubution of control
        cov_l_squar = (alpha1 * l)**2 + (alpha2 * (l-r))**2 # details is demon, make it right
        cov_r_squar = (alpha1 * r)**2 + (alpha2 * (l-r))**2
        
        
        particle_list = []
        for i in xrange(len(self.particles)):
        #"""# worst for loop, in python we can use for loop without
        #  use its index i to access its value"""
        #for particle in self.particles:
            #do the smaple of contral(gaussian model)
            l_sample = random.gauss(l,sqrt(cov_l_squar)) # hier second variance is stddev which is sqrt(variance)
            r_sample = random.gauss(r,sqrt(cov_r_squar))
            control_sample = [l_sample,r_sample]
            # predict the new particle
            particle_list.append(self.g(self.particles[i],control_sample,self.robot_width))
        self.particles = particle_list
        # Compute left and right variance.
        # alpha_1 is self.control_motion_factor.
        # alpha_2 is self.control_turn_factor.
        # Then, do a loop over all self.particles and construct a new
        # list of particles.
        # In the end, assign the new list of particles to self.particles.
        # For sampling, use random.gauss(mu, sigma). (Note sigma in this call
        # is the standard deviation, not the variance.)

    def print_particles(self, file_desc):
        """Prints particles to given file_desc output."""
        if not self.particles:
            return
        print >> file_desc, "PA",
        for p in self.particles:
            print >> file_desc, "%.0f %.0f %.3f" % p,
        print >> file_desc


if __name__ == '__main__':
    # Robot constants.
    scanner_displacement = 30.0
    ticks_to_mm = 0.349
    robot_width = 155.0

    # Filter constants.
    control_motion_factor = 0.35  # Error in motor control.
    control_turn_factor = 0.6  # Additional error due to slip when turning.

    # Generate initial particles. Each particle is (x, y, theta).
    number_of_particles = 300
    measured_state = (1850.0, 1897.0, 213.0 / 180.0 * pi)
    standard_deviations = (100.0, 100.0, 10.0 / 180.0 * pi)
    initial_particles = []
    for i in xrange(number_of_particles):
        initial_particles.append(tuple([
            random.gauss(measured_state[j], standard_deviations[j])
            for j in xrange(3)]))

    # Setup filter.
    pf = ParticleFilter(initial_particles,
                        robot_width, scanner_displacement,
                        control_motion_factor, control_turn_factor)

    # Read data.
    logfile = LegoLogfile()
    logfile.read("robot4_motors.txt")

    # Loop over all motor tick records.
    # This is the particle filter loop, with prediction and correction.
    f = open("particle_filter_predicted.txt", "w")
    for i in xrange(len(logfile.motor_ticks)):
        # Prediction.
        control = map(lambda x: x * ticks_to_mm, logfile.motor_ticks[i])
        pf.predict(control)

        # Output particles.
        pf.print_particles(f)

    f.close()
