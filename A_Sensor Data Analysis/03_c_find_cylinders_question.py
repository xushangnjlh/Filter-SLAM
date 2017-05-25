# For each cylinder in the scan, find its ray and depth.
# 03_c_find_cylinders
# XU Shang, 2016-10-10
from pylab import plot,show,scatter
from lego_robot import LegoLogfile

# Find the derivative in scan data, ignoring invalid measurements.
def compute_derivative(scan, min_dist):
    jumps = [0]
    
    for i in xrange(1, len(scan) - 1):
        f1 = scan[i+1]
        f2 = scan[i-1]

        if f1 > min_dist and f2 > min_dist:
            derivative = (f1 - f2)/2.0
        else:
            derivative = 0
            
        jumps.append(derivative)
        
    jumps.append(0)
    return jumps

# For each area between a left falling edge and a right rising edge,
# determine the average ray number and the average depth.
def find_cylinders(scan, scan_derivative, jump, min_dist):
    cylinder_list = []
    on_cylinder = False
    sum_ray, sum_depth, rays = 0.0, 0.0, 0

    for i in xrange(len(scan_derivative)):
        # --->>> Insert your cylinder code here.
        # Whenever you find a cylinder, add a tuple
        # (average_ray, average_depth) to the cylinder_list.
        if scan_derivative[i] < (-jump): 
            if rays > 1:
                cylinder_list.append((sum_ray/rays,sum_depth/rays))
            on_cylinder = True
            sum_ray, sum_depth, rays = 0.0, 0.0, 0
        
        if scan_derivative[i] > jump:
            on_cylinder = False
        
        if on_cylinder:
            if scan[i] > min_dist:
                rays = rays+1
                sum_ray = sum_ray+i
                sum_depth = sum_depth+scan[i]
        
    return cylinder_list


if __name__ == '__main__':

    minimum_valid_distance = 20.0
    depth_jump = 100.0

    # Read the logfile which contains all scans.
    logfile = LegoLogfile()
    logfile.read("robot4_scan.txt")

    # Pick one scan.
    scan = logfile.scan_data[8]

    # Find cylinders.
    der = compute_derivative(scan, minimum_valid_distance)
    
    cylinders = find_cylinders(scan, der, depth_jump,
                               minimum_valid_distance)

    # Plot results.
    plot(scan)
    plot(der)
    scatter([c[0] for c in cylinders], [c[1] for c in cylinders],
        c='r', s=200)
    show()
