import math

class CoordCalculations:
    #def __init__(self, )
    earth_radius = 6378100 # in meters

    @staticmethod
    def get_coords(lat1, lon1, distance, num_vertices):
        orig_dist = distance
        coords = []
        bearing = 0
        mult = 0

        coords.append({'lat': lat1, 'long': lon1})#, 'distance': orig_dist, 'bearing': bearing})

        for i in range(num_vertices):
            # update the distance as needed
            if (i > 1) and (i % 2) == 0:
                mult += orig_dist
            coords.append(CoordCalculations.calc_dest_coord(coords[i]['lat'], coords[i]['long'], distance + mult, bearing))
            bearing += 90

        return coords

    @staticmethod
    def calc_dest_coord(lat1, lon1, distance, bearing):
        lat2_rad = 0
        lon2_rad = 0

         # convert latitude, longitude, and bearing from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)

        # call respective direction method
        if bearing % 360 == 0:
            lat2_rad, lon2_rad = CoordCalculations.calc_north_coord(lat1_rad, lon1_rad, distance)
        elif bearing % 360 == 90:
            lat2_rad, lon2_rad = CoordCalculations.calc_east_coord(lat1_rad, lon1_rad, distance)
        elif bearing % 360 == 180:
            lat2_rad, lon2_rad = CoordCalculations.calc_south_coord(lat1_rad, lon1_rad, distance)
        elif bearing % 360 == 270:
            lat2_rad, lon2_rad = CoordCalculations.calc_west_coord(lat1_rad, lon1_rad, distance)
        else:
            return {}

        # convert latitude and longitude back to degrees
        lat2 = math.degrees(lat2_rad)
        lon2 = math.degrees(lon2_rad)
        coord = {'lat': lat2, 'long': lon2, 'distance': distance, 'bearing': bearing % 360}

        return coord

    @staticmethod
    def calc_north_coord(lat1, lon1, distance):
        lat2 = math.asin(math.sin(lat1) * math.cos(distance / CoordCalculations.earth_radius) + math.cos(lat1) * math.sin(distance / CoordCalculations.earth_radius) * 1)

        return lat2, lon1

    @staticmethod
    def calc_east_coord(lat1, lon1, distance):
        lon2 = lon1 + math.atan2(1 * math.sin(distance / CoordCalculations.earth_radius) * math.cos(lat1), math.cos(distance / CoordCalculations.earth_radius) - math.sin(lat1) ** 2)

        return lat1, lon2

    @staticmethod
    def calc_south_coord(lat1, lon1, distance):
        lat2 = math.asin(math.sin(lat1) * math.cos(distance / CoordCalculations.earth_radius) + math.cos(lat1) * math.sin(distance / CoordCalculations.earth_radius) * -1)

        return lat2, lon1


    @staticmethod
    def calc_west_coord(lat1, lon1, distance):
        lon2 = lon1 + math.atan2(-1 * math.sin(distance / CoordCalculations.earth_radius) * math.cos(lat1), math.cos(distance / CoordCalculations.earth_radius) - math.sin(lat1) ** 2)

        return lat1, lon2