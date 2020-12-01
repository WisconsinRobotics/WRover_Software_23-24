import math
import matplotlib.pyplot as plt
import scipy.ndimage as gaussian_smooth

class Histogram():
    sectors = [0]
    # Degrees the lidar can make measurements for
    LIDAR_VISION = 270
    # Degrees between any two data point
    ANGLE_STEP = 0.3332999980059538903

    def __init__(self, sector_count: int, threshold: float, vision_angle: int, data, smoothing: float):
        # Initialize the parameters as instance variables

        # Set the 'clear path' threshold
        self.threshold = threshold
        # Initialize a list of sectors of size sector_count
        self.sectors = [0] * sector_count
        # Copy the LIDAR data into the object
        self.data = data
        # Set the angles per sector constant
        # The histogram is 360 degrees, regardless of the size of the LIDAR data
        self.alpha = 360 / sector_count
        # Copy the vision angle
        self.vision_angle = vision_angle
        # Set the maximum range of the LIDAR
        # Correct maximum range of the LIDAR is about 50 meters, TODO: constant currently under tuning
        self.data.range_max = 52
        # Populate the histogram sectors with the LIDAR data & smooth
        self.populate_sectors(smoothing_constant=smoothing)

    # Angles of histogram go from 0-360 starting at the
    # right hand side and moving counter clockwise
    # LIDAR has 270 degrees of horizontal aperture
    # ranges list starts from rightmost angle to left most
    # LIDAR gives angular distance between measurements of 0.75 degrees
    # we want sectors starting from right hand side

    # Create the histogram sectors and smooth them using a Guassian filter
    def populate_sectors(self, smoothing_constant: float = 3) -> None:
        # number of measurements we need to skip from the start and the end,
        # based on our specific vision angle
        trim_measurements = int(
            math.ceil(
                ((self.LIDAR_VISION -
                  self.vision_angle) /
                 2.0) /
                math.degrees(
                    self.data.angle_increment)))

        # Start offset into the data based off of the used vision sectors
        start = trim_measurements + 1
        # Set the end offset into the data based off of the used vision sectors
        end = len(self.data.ranges) - trim_measurements
        # Set the readings per sector to be the angle per sector over the angle per reading of the sensor
        readings_per_sector = math.ceil(self.alpha / self.ANGLE_STEP)
        # Initialize the current sector being compiled and the number of data points in that sector's bin
        sector_num = 0
        data_count = 0
        
        # For each data point in the used data portion...
        for i in range(start, end):
            # Get the magnitude of the current reading, filtering out low level noise to 0
            magnitude = 0 if self.data.ranges[i] <= 0.0001 else self.get_magnitude(
                self.data.ranges[i])
            
            # If we've filled in all the sectors at this point...
            if sector_num >= len(self.sectors):
                # Exit the for loop
                break

            # Add the current magnitude to the current sector
            self.sectors[sector_num] += magnitude
            # Add one reading to the number of readings in the current sector
            data_count += 1
            # If we have maxed out the current number of readings in this sector...
            if data_count >= readings_per_sector:
                # Increment the sector being worked on for the next loop cycle
                sector_num += 1
                # Reset the number of readings for the next sector
                data_count = 0

        # Smooth the sector data using a Gaussian filter
        self.sectors = gaussian_smooth.gaussian_filter1d(self.sectors, smoothing_constant) 


    # Get the magnitude of the reading, scaled inverse to the distance of the reading
    # This allows obstacles further away to be regarded as less important in the current histogram calculations
    def get_magnitude(self, distance: float) -> float:
        # let the magnitude range from 0 - 1
         # the close a measurement is the greatest its magnitude
        # We want a = b (self.data.range_max * self.data.range_max)
        # we could try it with squared distance as well
        a = 1.0
        b = a / (self.data.range_max)  # * self.data.range_max)
        return a - b * distance  # * distance
