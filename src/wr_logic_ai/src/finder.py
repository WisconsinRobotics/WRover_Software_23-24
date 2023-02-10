from typing import List
import histogram
import math

import pickle

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
        sector_count: int,
        sector_angle: float,
        threshold: float,
        vision_angle: int,
        data,
        smoothing: float = 3) -> List[int]:

    # Get the smoothed histogram of the vision data
    hist = histogram.Histogram(
        sector_count,
        threshold,
        vision_angle,
        data,
        smoothing)

    # Write the sectors data to an output file for logging
    output_file = open('sectors.data', 'wb')
    pickle.dump(hist.sectors, output_file)
    output_file.close()

    # Initialize the target valley to the worst conditions so that it is immediately overriden
    valley_start = None
    best_valley = [0, len(hist.sectors) - 1]
    # The furthest a valley could every be from is 360 degrees since this is a circle
    best_distance = 361

    # For each sector in the histogram...
    for i in range(len(hist.sectors)):
        print(hist.sectors[i])

        # If the start of the valley hasn't been set yet...
        if valley_start is None:
            # ...and the sector is below the threshold...
            if hist.sectors[i] < threshold:
                # Start the valley at the current sector
                valley_start = i

        # If the start of the valley has been set and the current sector is above the threshold...
        elif hist.sectors[i] > threshold:
            # Get the effective distance between the target angle and the sector
            dist = get_target_distance(valley_start, i, target, sector_angle)

            # If current valley is closer to the target than the current best valley...
            if dist < best_distance:
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
        if dist < best_distance:
            # Set the best valley to the measured valley
            best_valley = [valley_start, i]

    # TODO: REVERSE DIRECTION IN CASE OF EMPTY VALLEY if best_valley is [None,
    # None]:
    # TODO: Implement play_navigation (drive in reverse w.r.t. log file)

    # Return the sectors defining the best valley
    return best_valley


# Gets the best angle to navigate to
def get_navigation_angle(
        target: int,
        sector_count: int,
        sector_angle: int,
        threshold: float,
        vision_angle: int,
        data,
        smoothing_constant:float = 3) -> float:

    # Get the best valley in the LIDAR data given our target angle
    best_valley = get_valley(
        target,
        sector_count,
        sector_angle,
        threshold,
        vision_angle,
        data,
        smoothing = smoothing_constant)
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
        border_sector = sector_count if border_sector > sector_count else border_sector

        # Aim for the center of this new max_valley valley (this helps avoid accidentally clipping an edge of the robot)
        return ((nearest_sector + border_sector) / 2.0) * sector_angle

    # If the valley is narrow...
    else:
        # Follow the probotcol as defined above for narrow valleys
        
        #Aim for the center of the valley
        return ((best_valley[0] + best_valley[1]) / 2.0) * sector_angle
