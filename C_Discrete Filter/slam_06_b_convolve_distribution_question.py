# Instead of moving a distribution, move (and modify) it using a convolution.
# 06_b_convolve_distribution
# XU Shang, 2015-10-28
from pylab import plot, show, ylim
from distribution import Distribution

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


if __name__ == '__main__':
    arena = (0,1000)

    # Move 3 times by 20.
    moves = [20] * 50

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
