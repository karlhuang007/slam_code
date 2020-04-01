# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement and measurement steps.
# 06_d_histogram_filter
# Claus Brenner, 28 NOV 2012
from pylab import plot, show, ylim
from distribution import *

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


if __name__ == '__main__':
    arena = (0,220)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 10

    # Measurement data. Assume (for now) that the measurement data
    # is correct. - This code just builds a cumulative list of the controls,
    # plus the start position.
    p = start_position
    measurements = []
    for c in controls:
        p += c
        measurements.append(p)

    # This is the filter loop.
    for i in xrange(len(controls)):
        # Move, by convolution. Also termed "prediction".
        control = Distribution.triangle(controls[i], 10)
        position = convolve(position, control)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='b', linestyle='steps')

        # Measure, by multiplication. Also termed "correction".
        measurement = Distribution.triangle(measurements[i], 10)
        position = multiply(position, measurement)
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             color='r', linestyle='steps')

    show()
