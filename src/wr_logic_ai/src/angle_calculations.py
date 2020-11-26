# The calculation method used in here is the Haversine formula.
# It already accounts for the curvature of the Earth.
# The calculations have been verified by Yash.
import math

class AngleCalculations:
    EARTH_RADIUS = 6378100

    #Declare a new AngleCalculations object with the current coordinates and target coordinates
    def __init__(self, clatitude: float, clongitude: float, glatitude: float, glongitude: float):
        # Set current position
        self.cur_lat = clatitude
        self.cur_long = clongitude
        # Set target position
        self.tar_lat = glatitude
        self.tar_long = glongitude
        # Set the reference points for the angle
        self.up = False
        self.right = False

    # Get the Great Circle distance between two latitudes
    def latitude_to_distance(self, lat1: float, lat2:float) -> float:
        # Set the reference 'up' if the target is further north than the current location
        self.up = self.cur_lat < self.tar_lat

        # Uses Haversine calculations to compute the Great Circle distance between two latitudes
        deltaLatitude = math.radians(math.fabs(lat2 - lat1))
        verticalDistance = math.sin(deltaLatitude / 2.0)**2
        angularDistance = 2 * math.atan2(math.sqrt(verticalDistance), math.sqrt(1.0 - verticalDistance))
        return angularDistance * self.EARTH_RADIUS

    # Get the Great Circle distance between two longitudes
    def longitude_to_distance(self, lon1: float, lon2: float) -> float:
        # Set the reference 'right' if the target is further on planar right than the current location
        self.right = self.cur_long < self.tar_long

        # Uses Haversine calculations to compute the Great Circle distance between two longitudes
        deltaLongitude = math.radians(math.fabs(lon2 - lon1))
        lateralDistance = math.cos(math.radians(self.cur_lat)) * math.cos(math.radians(self.tar_lat)) * math.sin(deltaLongitude / 2.0)**2
        angularDistance = 2 * math.atan2(math.sqrt(lateralDistance), math.sqrt(1.0 - lateralDistance))
        return angularDistance * self.EARTH_RADIUS

    # Get the Great Circle distance between this object's current and target locations
    def get_distance(self) -> float:
        # Since the contained functions compute great circle distance, the Euclidean distance formula will compute the right Great Circle composite distance
        return math.sqrt(self.latitude_to_distance(self.cur_lat, self.tar_lat)**2 + self.longitude_to_distance(self.cur_long, self.tar_long)**2)

    # Get the planar angle relative to planar East as the straight-line trajectory towards the goal
    def get_angle(self) -> float:
        # Use the Great Circle distances of the spherical triangle legs to get the angle (-90,90) to the goal coordinates
        angle = math.atan(self.latitude_to_distance(self.cur_lat, self.tar_lat) / self.longitude_to_distance(self.cur_long, self.tar_long))
        angle = math.degrees(angle)

        # Use the reference flags to move the angle to the right quadrant.
        # Angle in Q II
        if (not self.right and self.up):
            angle = 180.0 - angle

        # Angle in Q III
        elif (not self.right and not self.up):
            angle = angle + 180.0

        # Angle in Q IV
        elif (self.right and not self.up):
            angle = 360.0 - angle

        return angle

    # Get the target angle of the goal relative to some heading (that is relative to planar East)
    def get_target_angle(self, heading):
        # Move the heading to the standard angle interval
        heading = heading % 360
        # Get the angle of the target
        angle = self.get_angle()
        # Compute the target angle relative to our heading in the standard angle interval
        # TODO: Is the ternary operator taken care of by Python modulo?
        return angle - heading if angle >= heading else (360 - heading) + angle

