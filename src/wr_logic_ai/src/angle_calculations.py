import math
import numpy as np


class AngleCalculations:
    EARTH_RADIUS = 6378100
    SPHERICAL_EAST = [1,0,math.pi/2] # rho, theta, phi

    def __init__(self, clatitude, clongitude, glatitude, glongitude):
        self.cur_lat = clatitude
        self.cur_long = clongitude
        self.tar_lat = glatitude
        self.tar_long = glongitude
        self.up = False
        self.right = False
        self.down = False
        self.left = False

    def spherical_to_cartesian(coord: list) -> np.ndarray:
        rho = coord[0]
        theta = coord[1]
        phi = coord[2]
        return rho*np.array([math.cos(theta)*math.sin(phi), math.sin(theta)*math.sin(phi), math.cos(phi)])
    
    def get_angle(self):
        curr_coords = [1].append(list(map(lambda i: math.radians(i), [90 - self.cur_lat, self.cur_long])))
        targ_coords = [1].append(list(map(lambda i: math.radians(i), [90 - self.tar_lat, self.tar_long])))

        a = AngleCalculations.spherical_to_cartesian(SPHERICAL_EAST)
        b = AngleCalculations.spherical_to_cartesian(curr_coords)
        c = AngleCalculations.spherical_to_cartesian(targ_coords)

        n1 = np.cross(a, b)
        n2 = np.cross(b, c)

        angle = math.acos(np.dot(n1, n2)/(np.sqrt(np.dot(n1, n1) * np.sqrt(np.dot(n2, n2)))))
        
        self.up = self.cur_lat < self.tar_lat
        self.down = not self.up
        self.right = self.cur_long < self.tar_long
        self.left = not self.right
        
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

