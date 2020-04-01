# Comparison of the Kalman filter and the histogram filter.
# 06_f_kalman_vs_histogram_filter
# Claus Brenner, 29 NOV 2012
from distribution import *
from math import sqrt
from matplotlib.mlab import normpdf
from pylab import plot, show, ylim

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    d = distribution
    moved_distribution = Distribution(d.start()+delta,d.values[:])
    """when a function pass parameter through the parenthese in inner, we have to use 
     the data from the passed Obeject, rather than just create manutelly, because
     manutelly created data pass only in this specify case.
     how? find the class.py about the data extracted way."""
    #moved_distribution = Distribution.triangle(d.start()+1+delta,2)
    # --->>> Insert your code here.
    
    return moved_distribution  # Replace this by your own result.

    # --->>> Copy your previous code here.

    return distribution  # Replace this by your own result.


def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""

    # --->>> Copy your previous code here.
    multi_value = []
    Dist_List = []
    #d = Distribution()
    # --->>> Put your code here.
    """ enumerate() returns a tuple containing a count (from start which defaults 
    to 0) and the values obtained from iterating over iterable.
    the second way to construct a loop is for value in list, which take directally
    the value, no more list[i] needed to represent the value"""
    for i,a_val in enumerate(a.values):
        for b_val in b.values:
            multi_value.append(a_val*b_val)
        Dist_List.append(Distribution(a.start()+b.start()+i,multi_value[:]))
        """i hier is to move step every loop """
        multi_value = []# clear the list to avoid data confused
        """if you use a container in a loop, remenber to take care, if
        if the container should be clear after used?"""
    d = Distribution.sum(Dist_List[:])
    return d  # Replace this by your own result.
    return a  # Replace this by your own result.


def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""
    multi_value = []
    """two kurve which need one for loop can use this way to decide a start and end 
    point"""
    start = a.start() if a.start()>b.start() else b.start()
    stop = a.stop() if a.stop()<b.stop() else b.stop()
    
    for i in xrange(start,stop):
        multi_value.append(a.value(i)*b.value(i))
    d = Distribution(start,multi_value)
    Distribution.normalize(d)#the normalize function dont need to write ourself, just
    #use it, but my way kann 
    return d
    # --->>> Copy your previous code here.
    
    return a  # Modify this to return your result.
# Helpers.
#
class Density:
    def __init__(self, mu, sigma2):
        self.mu = float(mu)
        self.sigma2 = float(sigma2)

def histogram_plot(prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot(prediction.plotlists(*arena)[0], prediction.plotlists(*arena)[1],
         color='#C0C0FF', linestyle='steps', linewidth=5)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='#C0FFC0', linestyle='steps', linewidth=5)    
    plot(correction.plotlists(*arena)[0], correction.plotlists(*arena)[1],
         color='#FFC0C0', linestyle='steps', linewidth=5)

def kalman_plot(prediction, measurement, correction):
    """Helper to draw all curves in each filter step."""
    plot([normpdf(x, prediction.mu, sqrt(prediction.sigma2))
          for x in range(*arena)], color = 'b', linewidth=2)
    plot([normpdf(x, measurement.mu, sqrt(measurement.sigma2))
          for x in range(*arena)], color = 'g', linewidth=2)
    plot([normpdf(x, correction.mu, sqrt(correction.sigma2))
          for x in range(*arena)], color = 'r', linewidth=2)

#
# Histogram filter step.
#
def histogram_filter_step(belief, control, measurement):
    """Bayes filter step implementation: histogram filter."""
    # These two lines is the entire filter!
    prediction = convolve(belief, control)
    correction = multiply(prediction, measurement)

    return (prediction, correction)

#
# Kalman filter step.
#
def kalman_filter_step(belief, control, measurement):
    """Bayes filter step implementation: Kalman filter."""

    # --->>> Put your code here.
    
    # Prediction.
    prediction = Density(belief.mu + control.mu, belief.sigma2 + control.sigma2)  # hier a=1

    # Correction.
    K = prediction.sigma2/(prediction.sigma2+measurement.sigma2)
    Mu = prediction.mu+K*(measurement.mu-prediction.mu)
    Sigma2 = (1- K)*prediction.sigma2
    correction = Density(Mu,Sigma2)  # Replace
    
    return (prediction, correction)

#
# Main
#
if __name__ == '__main__':
    arena = (0,200)
    Dist = Distribution.gaussian  # Distribution.triangle or Distribution.gaussian.

    # Start position. Well known, so the distribution is narrow.
    position = Dist(10, 1)      # Histogram
    position_ = Density(10, 1)  # Kalman

    # Controls and measurements.
    controls = [ Dist(40, 10), Dist(70, 10) ]               # Histogram
    controls_ = [ Density(40, 10**2), Density(70, 10**2) ]  # Kalman
    measurements = [ Dist(60, 10), Dist(140, 20) ]               # Histogram
    measurements_ = [ Density(60, 10**2), Density(140, 20**2) ]  # Kalman

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Histogram
        (prediction, position) = histogram_filter_step(position, controls[i], measurements[i])
        histogram_plot(prediction, measurements[i], position)
        # Kalman
        (prediction_, position_) = kalman_filter_step(position_, controls_[i], measurements_[i])
        kalman_plot(prediction_, measurements_[i], position_)

    ylim(0.0, 0.06)
    show()
