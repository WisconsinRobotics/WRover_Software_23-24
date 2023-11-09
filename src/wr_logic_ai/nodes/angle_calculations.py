"""@file
@defgroup wr_logic_ai_longrange_ai
@{
@defgroup wr_logic_ai_longrange_ai_angle_calculations Angle Calculations
@brief Helper class for angle calculation
@details Package of functions for angle calculations using the Haversine formula. Takes account for the 
curvature of the Earth.
@{
"""

import math
import rospy


class AngleCalculations:
    """
    Helper class containing all functions for angle calculation
    """

    EARTH_RADIUS_METERS = 6378100

    def __init__(
        self, clatitude: float, clongitude: float, glatitude: float, glongitude: float
    ):
        """
        Declare a new AngleCalculations object with the current coordinates and target coordinates

        @param clatitude (float): Current latitude of the rover
        @param clongitude (float): Current longitude of the rover
        @param glatitude (float): Target destination latitude
        @param glongitude (float): Target destination longitude
        """
        # Set current position
        self.cur_lat = clatitude
        self.cur_long = clongitude
        # Set target position
        self.tar_lat = glatitude
        self.tar_long = glongitude
        # Set the reference points for the angle
        self.up = False
        self.right = False

    def latitude_to_distance(self, lat1: float, lat2: float) -> float:
        """
        Get the Great Circle distance between two latitudes

        @param lat1 (float): First latitude coordinate
        @param lat2 (float): Second latitude coordinate
        @return float: The Great Circle distance between the given latitudes
        """

        # Set the reference 'up' if the target is further north than the current location
        self.up = self.cur_lat < self.tar_lat

        # Uses Haversine calculations to compute the Great Circle distance between two latitudes
        deltaLatitude = math.radians(math.fabs(lat2 - lat1))
        verticalDistance = math.sin(deltaLatitude / 2.0) ** 2
        angularDistance = 2 * math.atan2(
            math.sqrt(verticalDistance), math.sqrt(1.0 - verticalDistance)
        )
        if self.up:
            return angularDistance * self.EARTH_RADIUS_METERS
        else:
            return -1 * angularDistance * self.EARTH_RADIUS_METERS

    def longitude_to_distance(self, lon1: float, lon2: float) -> float:
        """
        Get the Great Circle distance between two longitudes

        @param lon1 (float): First longitude coordinate
        @param lon2 (float): Second longitude coordinate
        @return float: The Great Circle distance between the given longitudes
        """

        # Set the reference 'right' if the target is further on planar right than the current location
        self.right = self.cur_long < self.tar_long

        # Uses Haversine calculations to compute the Great Circle distance between two longitudes
        deltaLongitude = math.radians(math.fabs(lon2 - lon1))
        lateralDistance = (
            math.cos(math.radians(self.cur_lat))
            * math.cos(math.radians(self.tar_lat))
            * math.sin(deltaLongitude / 2.0) ** 2
        )
        angularDistance = 2 * math.atan2(
            math.sqrt(lateralDistance), math.sqrt(1.0 - lateralDistance)
        )
        if self.right:
            return angularDistance * self.EARTH_RADIUS_METERS
        else:
            return -1 * angularDistance * self.EARTH_RADIUS_METERS

    def get_distance(self) -> float:
        """
        Get the Great Circle distance between this object's current and target locations

        @return float: The Great Circle distance between this object's current and target locations
        """

        # Since the contained functions compute great circle distance, the Euclidean distance formula will compute the right Great Circle composite distance
        return math.sqrt(
            self.latitude_to_distance(self.cur_lat, self.tar_lat) ** 2
            + self.longitude_to_distance(self.cur_long, self.tar_long) ** 2
        )

    def get_angle(self) -> float:
        """
        Get the planar angle relative to planar East as the straight-line trajectory towards the goal

        @return float: The planar angle relative to planar East as the straight-line trajectory towards the goal
        """

        # Use the Great Circle distances of the spherical triangle legs to get the angle (-90,90) to the goal coordinates
        angle = math.atan2(
            self.latitude_to_distance(self.cur_lat, self.tar_lat),
            self.longitude_to_distance(self.cur_long, self.tar_long),
        )
        angle = math.degrees(angle)
        # rospy.loginfo(f"latitude to distance: {self.latitude_to_distance(self.cur_lat, self.tar_lat)}")
        # rospy.loginfo(f"longitude to distance: {self.longitude_to_distance(self.cur_long, self.tar_long)}")
        # rospy.loginfo(f"angle: {angle}")

        # #Setting up in which quadrant we are
        # if self.cur_lat < self.tar_lat:
        #     self.up = True
        # else:
        #     self.up = False
        # if self.cur_long < self.tar_long:
        #     self.right = True
        # else:
        #     self.right = False

        # # Use the reference flags to move the angle to the right quadrant.
        # # Angle in Q II
        # if (not self.right and self.up):
        #     angle = 180.0 - angle

        # # Angle in Q III
        # elif (not self.right and not self.up):
        #     angle = angle + 180.0

        # # Angle in Q IV
        # elif (self.right and not self.up):
        #     angle = 360.0 - angle

        return angle

    def get_target_angle(self, heading):
        """
        Get the target angle of the goal relative to some heading (that is relative to planar East)

        @param heading: The current heading of the rover
        @return float: The target angle relative to our current heading in the standard angle interval
        """

        # Move the heading to the standard angle interval
        heading = heading % 360
        # Get the angle of the target
        angle = self.get_angle()
        # Compute the target angle relative to our heading in the standard angle interval
        # TODO: Is the ternary operator taken care of by Python modulo?
        return angle - heading if angle >= heading else (360 - heading) + angle


## @}
## @}
