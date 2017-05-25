# Plot a scan of the robot using matplotlib.
# 03_a_plot_scan
# XU Shang, 2016-10-13
from pylab import plot,show
from lego_robot import LegoLogfile

# Read the logfile which contains all scans.
logfile = LegoLogfile()
logfile.read("robot4_scan.txt")

# Plot one scan.
plot(logfile.scan_data[10])

show()
