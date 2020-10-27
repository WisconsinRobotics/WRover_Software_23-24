import math
import matplotlib.pyplot as plt


class Histogram():
    sectors = [0]
    # Degrees the lidar can make measurements for
    LIDAR_VISION = 270
    # Degrees between any two data point
    ANGLE_STEP = 0.3332999980059538903

    def __init__(self, sector_count, threshold, vision_angle, data):
        self.threshold = threshold
        self.sectors = [0] * sector_count
        self.data = data
        self.alpha = 360 / sector_count
        self.vision_angle = vision_angle
        # correct maximum range of the lidar is about 50 meters
        self.data.range_max = 52
        self.populate_sectors()

    # Angles of histogram go from 0-360 starting at the
    # right hand side and moving counter clockwise
    # LIDAR has 270 degrees of horizontal aperture
    # ranges list starts from rightmost angle to left most
    # LIDAR gives angular distance between measurements of 0.75 degrees
    # we want sectors starting from right hand side
    def populate_sectors(self):
        # number of measurements we need to skip from the start and the end,
        # based on our specific vision angle
        trim_measurements = int(
            math.ceil(
                ((self.LIDAR_VISION -
                  self.vision_angle) /
                 2.0) /
                math.degrees(
                    self.data.angle_increment)))
        start = trim_measurements + 1
        end = len(self.data.ranges) - trim_measurements
        readings_per_sector = math.ceil(self.alpha / self.ANGLE_STEP)
        sector_num = 0
        data_count = 0
        for i in range(start, end):
            magnitude = 0 if self.data.ranges[i] <= 0.0001 else self.get_magnitude(
                self.data.ranges[i])
            if sector_num >= len(self.sectors):
                break

            self.sectors[sector_num] += magnitude
            data_count += 1
            if data_count >= readings_per_sector:
                sector_num += 1
                data_count = 0

        old_sectors = self.sectors[:]
        for i in range(len(self.sectors)):
            self.sector_smoothing(i, old_sectors)

    def sector_smoothing(self, sector_number, old_sectors):
        l = 3
        start = sector_number - l
        end = sector_number + l + 1
        start = 0 if start < 0 else start
        end = len(old_sectors) if end > len(old_sectors) else end
        num = 0
        for i in range(start, end):
            constant = abs((sector_number - l) - i) + 1
            constant_2 = abs((sector_number + l) - i) + 1
            constant = min(constant, constant_2)
            num += constant * old_sectors[i]

        self.sectors[sector_number] = num / (end - start)

    # let the magnitude range from 0 - 1
    # the close a measurement is the greatest its magnitude
    def get_magnitude(self, distance):
        # We want a = b (self.data.range_max * self.data.range_max)
        # we could try it with squared distance as well
        a = 1.0
        b = a / (self.data.range_max)  # * self.data.range_max)
        return a - b * distance  # * distance
