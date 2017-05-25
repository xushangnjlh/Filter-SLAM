# Histogram implementation of a bayes filter - combines
# convolution and multiplication of distributions, for the
# movement(prediction) and measurement(correction) steps.
# 06_d_histogram_filter
# 2015-9
from pylab import plot, show, ylim
from distribution import *

def move(distribution, delta):
    """Returns a Distribution that has been moved (x-axis) by the amount of
       delta."""
    return Distribution(distribution.offset + delta, distribution.values)

def convolve(a, b):
    """Convolve distribution a and b and return the resulting new distribution."""
    # pay attention to the index
    offset = a.offset + b.offset + 1 - (len(b.values)-1)/2
    c = Distribution(offset,[0]*(len(b.values)+len(a.values)-1))
    for m in xrange(0,len(a.values)):
        for n in xrange(0,len(b.values)):
            c.values[m+n] = c.values[m+n] + a.values[m]*b.values[n]
    return c  # Replace this by your own result.


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
    arena = (0,2200)

    # Start position. Exactly known - a unit pulse.
    start_position = 10
    position = Distribution.unit_pulse(start_position)
    plot(position.plotlists(*arena)[0], position.plotlists(*arena)[1],
         linestyle='steps')

    # Movement data.
    controls  =    [ 20 ] * 100

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
