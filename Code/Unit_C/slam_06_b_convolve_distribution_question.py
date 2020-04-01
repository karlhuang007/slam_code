# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""

    d = distribution
    moved_distribution = Distribution(d.start()+delta,d.values[:])
    # when a function pass parameter through the parenthese in inner, we have to use 
    # the data from the passed Obeject, rather than just create manutelly, because
    # manutelly created data pass only in this specify case.
    # how? find the class.py about the data extracted way.
    #moved_distribution = Distribution.triangle(d.start()+1+delta,2)
    # --->>> Insert your code here.
    
    return moved_distribution  # Replace this by your own result.

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution.
        COnvolve is actually a move from a-distribution to b-distribution"""
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


if __name__ == '__main__':
    arena = (0,100)

    # Move 3 times by 20.
    moves = [20] * 3

    # Start with a known position: probability 1.0 at position 10.
    position = Distribution.unit_pulse(10)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Now move and plot.
    for m in moves:
        move_distribution = Distribution.triangle(m, 2)
        position = convolve(position, move_distribution)
        
        plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
             linestyle='steps')

    ylim(0.0, 1.1)
    show()
