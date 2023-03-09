from typing import List
import math
import scipy.ndimage as gaussian_smooth
import rospy
from sensor_msgs.msg import LaserScan 
from copy import deepcopy

import pickle

ROVER_WIDTH = 0

# TESTING
scan_rviz_pub = rospy.Publisher('/scan_rviz', LaserScan, queue_size=10)

def calculate_angle_to_check(t: float) -> int:
    return math.degrees(math.acos((2*t*t - ROVER_WIDTH*ROVER_WIDTH)/(2*t*t)))/2

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

# Returns if valley is wide enough for rover to pass
def check_valley_width(left_sector: int, right_sector: int, left_sector_dist: int, right_sector_dist: int, sector_angle: int) -> bool:
    angle = (right_sector - left_sector) * sector_angle
    valley_width = math.sqrt(left_sector_dist * left_sector_dist + right_sector_dist * right_sector_dist - 2 * left_sector_dist * right_sector_dist * math.cos(math.radians(angle)))
    return (valley_width > ROVER_WIDTH)

# Transforms the raw lidar data from compass coordinates (0 at north, clockwise) to math coordinates (0 at east, counterclockwise)
def offset_lidar_data(data, sector_angle, is_rviz = False):
    offset_data = [0] * len(data)
    if is_rviz:
        for i in range(len(data)):
            offset_data[(i + math.floor(90 / sector_angle) + math.floor(len(data) / 2)) % len(data)] = data[i]
    else:
        for i in range(len(data)):
            offset_data[(i + math.floor(90 / sector_angle)) % len(data)] = data[i]
    return offset_data

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

    angle_to_check = calculate_angle_to_check(threshold)
    sector_count = len(data.ranges)
    rviz_data = deepcopy(data)
    rviz_data.ranges = offset_lidar_data(data.ranges, sector_angle, True)
    scan_rviz_pub.publish(rviz_data)

    hist = offset_lidar_data(gaussian_smooth.gaussian_filter1d(data.ranges, smoothing), sector_angle)

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
        print(hist[len(hist) // 2])
        # If the start of the vaprint(data.angle_increment)lley hasn't been set yet...
        if valley_start is None:
            # ...and the sector is below the threshold...
            if hist[i] > threshold:
                if(checkWidth(i ,threshold, hist, angle_to_check)):
                    valley_start = i
                # Start the valley at the current sector        
                
        # If the start of the valley has been set and the current sector is above the threshold...
        elif checkWidth(i ,threshold, hist, angle_to_check):
            # Get the effective distance in degrees between the target angle and the sector
            dist = get_target_distance(valley_start, i, target, sector_angle) # dist is the degree from target to current sector
            # If current valley is closer to the target than the current best valley...
            if dist < best_distance and check_valley_width(valley_start, i, hist[valley_start], hist[i], sector_angle):
                # Replace the best distance and best valley
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
        if dist < best_distance and check_valley_width(valley_start, i, hist[valley_start], hist[i], sector_angle):
            # Set the best valley to the measured valley
            # Replace the best distance and best valley
            best_distance = dist
            best_valley = [valley_start, i]

    #Make calculations to check if best valley is enough for robot to go through
    

    # TODO: REVERSE DIRECTION IN CASE OF EMPTY VALLEY if best_valley is [None,
    # None]:
    # TODO: Implement play_navigation (drive in reverse w.r.t. log file)

    # Return the sectors defining the best valley
    
    
    return best_valley

def checkWidth(sector: int, threshold: float, hist: List, angle_to_check: int, sector_angle: float) -> bool:
    for i in range(sector - angle_to_check / sector_angle, sector + angle_to_check / sector_angle):
        if(hist[i % len(hist)] < threshold):
            return True
    return False

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
    print("best valley: " + str(best_valley[0]) + " " + str(best_valley[1]))

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