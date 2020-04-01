# Multiply a distribution by another distribution.
# 06_c_multiply_distribution
# Claus Brenner, 26 NOV 2012
from pylab import plot, show
from distribution import *

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""
    """self-method to normalize, it work also"""
#    sum_value = 0
#    multi_value = []
#    for i in range(a.start(),b.stop()-1):#this range is right to do a sum() function
#        sum_value += a.value(i)*b.value(i)
#    for i in range(a.start(),b.stop()-1):
#        if sum_value != 0:
#            multi_value.append((a.value(i)*b.value(i))/sum_value)
#    d = Distribution(a.start(),multi_value)
#    return d
    
    """use given normalize function """
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
    
    
    
    
    
    
    
    
    
    
#           sum_value = sum_value+(a_val*b_val)
#    if a.start()>b.start() and a.stop()<b.stop():
#        for i in xrange(a.start(),a.stop()-1):
#            sum_value += a.value(i)*b.value(i)
##            multi_value.append((a.value(i)*b.value(i))/sum_value)
#        for i in xrange(a.start(),a.stop()-1):
#            multi_value.append((a.value(i)*b.value(i))/sum_value)
#        d = Distribution(a.start(),multi_value)
#    elif a.start()<b.start() and a.stop()>b.stop():
#        for i in xrange(b.start(),b.stop()-1):
#            sum_value += a.value(i)*b.value(i)
#        for i in xrange(b.start(),b.stop()-1):
#            multi_value.append((a.value(i)*b.value(i))/sum_value)
#        d = Distribution(b.start(),multi_value)
#    elif a.start()<b.start() and a.stop()<b.stop():
#        for i in xrange(b.start(),a.stop()-1):
#            sum_value += a.value(i)*b.value(i)
#        for i in xrange(b.start(),a.stop()-1):
#            multi_value.append((a.value(i)*b.value(i))/sum_value)
#        d = Distribution(b.start(),multi_value)
#    elif a.start()>b.start() and a.stop()>b.stop():
#        for i in xrange(a.start(),b.stop()-1):
#            sum_value += a.value(i)*b.value(i)
#        for i in xrange(a.start(),b.stop()-1):
#            multi_value.append((a.value(i)*b.value(i))/sum_value)
#        d = Distribution(a.start(),multi_value)
    # --->>> Put your code here.
    
      # Modify this to return your result.


if __name__ == '__main__':
    arena = (0,1000)

    # Here is our assumed position. Plotted in blue.
    position_value = 400
    position_error = 100
    position = Distribution.triangle(position_value, position_error)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         color='b', linestyle='steps')

    # Here is our measurement. Plotted in green.
    # That is what we read from the instrument.
    measured_value = 410
    measurement_error = 200
    measurement = Distribution.triangle(measured_value, measurement_error)
    plot(measurement.plotlists(*arena)[0], measurement.plotlists(*arena)[1],
         color='g', linestyle='steps')

    # Now, we integrate our sensor measurement. Result is plotted in red.
    position_after_measurement = multiply(position, measurement)
    plot(position_after_measurement.plotlists(*arena)[0],
         position_after_measurement.plotlists(*arena)[1],
         color='r', linestyle='steps')

    show()
