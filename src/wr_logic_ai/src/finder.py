import histogram
import math

import pickle

# returns distance (in degrees) from sector parameter to target
# returns 0 if sector parameter contains target


def get_target_distance(sector_i, sector_f, target, sector_angle):
    # GETTING NONE FROM SECTOR VARIABLES
    if target < sector_i:
        return (sector_i - target) * sector_angle
    if target > sector_f:
        return (target - sector_f) * sector_angle
    return 0

# number of consecutive sectors where POD is below threshold


def is_wide_valley(sector_i, sector_f, max_valley):
    return 1 + sector_f - sector_i > max_valley

# represent a valley as an ordered pair as in (start sector, end sector)
# iterate through sector array to add valleys to new candidate valleys list
# from candidate valleys choose the one that is closest to the target sector
# valleys of any length should be valid since they can only come from smoothing function of adjacent areas
# and correct thresholding


def get_valley(
        target,
        sector_count,
        sector_angle,
        threshold,
        vision_angle,
        data,
        smoothing=3):
    hist = histogram.Histogram(
        sector_count,
        threshold,
        vision_angle,
        data,
        smoothing)  # TODO histogram and parameters
    output_file = open('sectors.data', 'wb')
    pickle.dump(hist.sectors, output_file)
    output_file.close()

    valley_start = None
    best_valley = [0, len(hist.sectors) - 1]
    # the furthest a valley could every be from is 360 degrees since this is a
    # circle
    best_distance = 361
    for i in range(len(hist.sectors)):
        if valley_start is None:
            if hist.sectors[i] < threshold:
                valley_start = i
        elif hist.sectors[i] > threshold:
            dist = get_target_distance(valley_start, i, target, sector_angle)
            if dist < best_distance:
                best_distance = dist
                best_valley = [valley_start, i]

            valley_start = None

    if valley_start is not None:
        dist = get_target_distance(
            valley_start, sector_count, target, sector_angle)
        if dist < best_distance:
            best_valley = [valley_start, i]

    # TODO: REVERSE DIRECTION IN CASE OF EMPTY VALLEY if best_valley is [None,
    # None]:
    return best_valley


# Gets the best angle to navigate to
def get_navigation_angle(
        target,
        sector_count,
        sector_angle,
        threshold,
        vision_angle,
        data,
        smoothing_constant = 3):
    best_valley = get_valley(
        target,
        sector_count,
        sector_angle,
        threshold,
        vision_angle,
        data,
        smoothing = smoothing_constant)
    # research suggest that the biggest valley should not be bigger than 90
    # degrees
    max_valley = int(90 / sector_angle)
    # if target and best valley are the same
    if get_target_distance(
            best_valley[0],
            best_valley[1],
            target,
            sector_angle) == 0:
        return target * sector_angle
    elif is_wide_valley(best_valley[0], best_valley[1], max_valley):
        # valley is big, we just set target at the closest safe angle
        nearest_sector = best_valley[1] if target > best_valley[1] else best_valley[0]
        border_sector = best_valley[1] - \
            max_valley if target > best_valley[1] else best_valley[0] + max_valley
        border_sector = 0 if border_sector < 0 else border_sector
        border_sector = sector_count if border_sector > sector_count else border_sector
        return ((nearest_sector + border_sector) / 2.0) * sector_angle

    else:
        # valley is small, centers the robot in the middle of the valley
        return ((best_valley[0] + best_valley[1]) / 2.0) * sector_angle
