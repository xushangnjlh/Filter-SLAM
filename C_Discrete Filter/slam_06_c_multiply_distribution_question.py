# Multiply a distribution by another distribution, which is the basic for Bayes Filter
# 06_c_multiply_distribution
# XU Shang, 2015-10-28
from pylab import plot, show
from distribution import *

def multiply(a, b):
    """Multiply two distributions and return the resulting distribution."""
    begin = min(a.offset,a.offset+len(a.values),b.offset,b.offset+len(b.values))
    end = max(a.offset,a.offset+len(a.values),b.offset,b.offset+len(b.values))
    c = Distribution(begin,[0]*(end-begin+1))
    for i in xrange(begin,end+1):
        c.values[i-begin] = a.value(i)*b.value(i)
    c.normalize()
    return c  


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
    measured_value = 610
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
