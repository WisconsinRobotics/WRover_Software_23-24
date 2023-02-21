from typing import List
#import histogram
import math
import scipy.ndimage as gaussian_smooth
import rospy
from sensor_msgs.msg import LaserScan 

import pickle

MAX_WINDOW_DISTANCE = 2

prevData = []

# rospy.init_node('publish_altered_histogram', anonymous=False)
distanceData = rospy.Publisher('/scan', LaserScan, queue_size=10)
laser2 = LaserScan()
laser2.angle_max = 2 * math.pi
laser2.angle_increment = math.pi / 180
laser2.time_increment = 0
laser2.scan_time = 1
laser2.range_min = 0
laser2.range_max = 150
laser2.header.frame_id = "map"
laser2.intensities = []

# Returns angular distance (in degrees) from sector parameter to target
# Returns 0 if sector parameter contains target
# TODO: Is there a cleaner way to write the logic?
def get_target_distance(sector_i: int, sector_f: int, target: int, sector_angle: float) -> float:
    # Compute if the target is under the starting sector
    if target < sector_i:
        # Return angular distance between target and starting sector
        return (sector_i - target) * sector_angle
    # Compute if the target is above the last sector
    if target > sector_f:
        # Return the angular distance between the target and last sector
        return (target - sector_f) * sector_angle
    # Otherwise, the target is within the sector bounds
    # Angular distance is then 0
    return 0


# Returns whether or not a valley is as wide or wider than the recommended max valley size
def is_wide_valley(sector_i: int, sector_f: int, max_valley: int) -> bool:
    #TODO: Cleaner way to write this?
    return 1 + sector_f - sector_i > max_valley

# represent a valley as an ordered pair as in (start sector, end sector)
# iterate through sector array to add valleys to new candidate valleys list
# from candidate valleys choose the one that is closest to the target sector
# valleys of any length should be valid since they can only come from smoothing function of adjacent areas
# and correct thresholding

# Get the 'best' valley from the LIDAR data.  The valley is formatted as [Start Sector, End Sector]
def get_valley(
        target: int,
        sector_angle: float,
        threshold: float,
        data,
        smoothing: float = 3) -> List[int]:
    global prevData

    sector_count = len(data.ranges)

    # Get the smoothed histogram of the vision data
    if len(prevData) == 0:
        prevData = [0] * sector_count
    
    rangesDelayed = [0] * sector_count
    for i in range(sector_count):
        rangesDelayed[i] = data.ranges[i] - prevData[i] / 2
    prevData = data.ranges

    hist = gaussian_smooth.gaussian_filter1d(data.ranges, smoothing)
    hist2 = gaussian_smooth.gaussian_filter1d(rangesDelayed, smoothing)

    laser2.ranges = hist2

    # Write the sectors data to an output file for logging
    output_file = open('sectors.data', 'wb')
    pickle.dump(hist, output_file)
    output_file.close()

    # Initialize the target valley to the worst conditions so that it is immediately overriden
    valley_start = None
    best_valley = [0, len(hist) - 1]
    # The furthest a valley could every be from is 360 degrees since this is a circle
    best_distance = 361

    # For each sector in the histogram...
    for i in range(len(hist)):
        # print(hist[i])

        # If the start of the valley hasn't been set yet...
        if valley_start is None:
            # ...and the sector is below the threshold...
            if hist[i] > threshold:
                # Start the valley at the current sector
                valley_start = i 


        # If the start of the valley has been set and the current sector is above the threshold...
        elif hist[i] < threshold:
            # Get the effective distance in degrees between the target angle and the sector
            dist = get_target_distance(valley_start, i, target, sector_angle) # dist is the degree from target to current sector
            # If current valley is closer to the target than the current best valley...
            if dist < best_distance:
                # Replace the best distance and best valley
                y = hist[i]
                x = hist[valley_start]
                angle = (i - valley_start)*sector_angle
                valley_width = math.sqrt(y*y+x*x-2*x*y*math.cos(math.radians(angle)))
                if valley_width > 2:
                    best_distance = dist
                    best_valley = [valley_start, i]

            # Since we are above the threshold limit, end the current valley
            valley_start = None

    # If a valley was started near the end of the vision range...
    if valley_start is not None:
        # Check the distance between the target angle and a valley starting at the recorded position and ending at the end of the view frame
        dist = get_target_distance(
            valley_start, sector_count, target, sector_angle)
        
        # If that distance was better than the current best sector distance...
        if dist < best_distance:
            # Set the best valley to the measured valley
           # Replace the best distance and best valley
            y = hist[i]
            x = hist[valley_start]
            angle = (i - valley_start)*sector_angle
            valley_width = math.sqrt(y*y+x*x-2*x*y*math.cos(math.radians(angle)))
            if valley_width > 2:
                best_distance = dist
                best_valley = [valley_start, i]

    #Make calculations to check if best valley is enough for robot to go through
    

    # TODO: REVERSE DIRECTION IN CASE OF EMPTY VALLEY if best_valley is [None,
    # None]:
    # TODO: Implement play_navigation (drive in reverse w.r.t. log file)

    # Return the sectors defining the best valley
    return best_valley


# Gets the best angle to navigate to
def get_navigation_angle(
        target: int,
        threshold: float,
        data,
        smoothing_constant: float = 3) -> float:

    sector_angle = math.degrees(data.angle_increment)

    # Get the best valley in the LIDAR data given our target angle
    best_valley = get_valley(
        target,
        sector_angle,
        threshold,
        data,
        smoothing_constant)
    #print("best valley: " + str(best_valley[0]) + " " + str(best_valley[1]))

    # Define the difference between 'wide' and 'narrow' valleys
    # For wide valleys, we want to drive on the edge of the valley
    #   i.e. Don't drive to the middle of a field just to avoid a pole
    # For narrow valleys, we want to drive in the middle of a valley
    #   i.e. We don't want to cut it too close on a doorway

    # research suggest that the biggest valley should not be bigger than 90
    # degrees
    max_valley = int(90 / sector_angle)
    # If the target is already in the best valley...
    if get_target_distance(
            best_valley[0],
            best_valley[1],
            target,
            sector_angle) == 0:

        # Report the current target angle; no adjustment needed
        #print("target * sector_angle = " + str(target * sector_angle))
        return target * sector_angle

    # If the valley is wide...
    elif is_wide_valley(best_valley[0], best_valley[1], max_valley):
        # Follow the protocol as defined above for wide valleys
        
        # Get the nearest edge of the sector
        nearest_sector = best_valley[1] if target > best_valley[1] else best_valley[0]
        
        # Construct a second border edge to make the valley of size max_valley
        border_sector = best_valley[1] - \
            max_valley if target > best_valley[1] else best_valley[0] + max_valley
        # Ensure that this new border edge is within the bounds of allowable sectors
        border_sector = 0 if border_sector < 0 else border_sector
        border_sector = len(data.ranges) if border_sector > len(data.ranges) else border_sector

        # Aim for the center of this new max_valley valley (this helps avoid accidentally clipping an edge of the robot)
        return ((nearest_sector + border_sector) / 2.0) * sector_angle

    # If the valley is narrow...
    else:
        # Follow the probotcol as defined above for narrow valleys
        
        #Aim for the center of the valley
        return ((best_valley[0] + best_valley[1]) / 2.0) * sector_angle

#while not rospy.is_shutdown():
#    
#    #rospy.loginfo(nav)
#    distanceData.publish(laser2)
