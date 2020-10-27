import math

class AngleCalculations:
    EARTH_RADIUS = 6378100

    def __init__(self, clatitude, clongitude, glatitude, glongitude):
        self.cur_lat = clatitude
        self.cur_long = clongitude
        self.tar_lat = glatitude
        self.tar_long = glongitude
        self.up = False
        self.right = False
        self.down = False
        self.left = False


    def latitude_to_distance(self, lat1, lat2):
        if(self.cur_lat < self.tar_lat):
            self.up = True
        else:
            self.down = True

        deltaLatitude = math.radians(math.fabs(lat2 - lat1))
        verticalDistance = math.pow(math.sin(deltaLatitude / 2.0), 2)
        angularDistance = 2 * math.atan2(math.sqrt(verticalDistance), math.sqrt(1.0 - verticalDistance))
        return angularDistance * self.EARTH_RADIUS

    def longitude_to_distance(self, lon1, lon2):
        if(self.cur_long < self.tar_long):
            self.right = True
        else:
            self.left = True;

        deltaLongitude = math.radians(math.fabs(lon2 - lon1))
        lateralDistance = math.cos(math.radians(self.cur_lat)) * math.cos(math.radians(self.tar_lat)) * math.pow(math.sin(deltaLongitude / 2.0), 2)
        angularDistance = 2 * math.atan2(math.sqrt(lateralDistance), math.sqrt(1.0 - lateralDistance))
        return angularDistance * self.EARTH_RADIUS

    def get_distance(self):
        return math.sqrt(math.pow(self.latitude_to_distance(self.cur_lat, self.tar_lat), 2) + math.pow(self.longitude_to_distance(self.cur_long, self.tar_long), 2))

    def get_angle(self):
        angle = math.atan(self.latitude_to_distance(self.cur_lat, self.tar_lat) / self.longitude_to_distance(self.cur_long, self.tar_long))
        angle = math.degrees(angle)
        if (self.left and self.up):
            angle = 180.0 - angle
        elif (self.left and self.down):
            angle = angle + 180.0
        elif (self.right and self.down):
            angle = 360.0 - angle

        return angle

    def get_target_angle(self, heading):
        heading = heading % 360
        angle = self.get_angle()
        return angle  - heading if angle >= heading else (360 - heading) + angle

