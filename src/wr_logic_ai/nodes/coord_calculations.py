import math

class CoordCalculations:
    #def __init__(self, )
    earth_radius = 6378100 # in meters

    @staticmethod
    def get_coords(curr_lat, curr_long, distance, num_vertices):
        orig_dist = distance
        coords = []
        bearing = 0
        mult = 0

        coords.append({'lat': curr_lat, 'long': curr_long})#, 'distance': orig_dist, 'bearing': bearing})

        for i in range(num_vertices):
            # update the distance as needed
            if (i > 1) and (i % 2) == 0:
                mult += orig_dist
            coords.append(CoordCalculations.calc_dest_coord(coords[i]['lat'], coords[i]['long'], distance + mult, bearing))
            bearing += 90

        return coords

    @staticmethod
    def calc_dest_coord(curr_lat, curr_long, distance, bearing):
        target_lat_rad = 0
        target_long_rad = 0

         # convert latitude, longitude, and bearing from degrees to radians
        curr_lat_rad = math.radians(curr_lat)
        curr_long_rad = math.radians(curr_long)

        # call respective direction method
        if bearing % 360 == 0:
            target_lat_rad, target_long_rad = CoordCalculations.calc_north_coord(curr_lat_rad, curr_long_rad, distance)
        elif bearing % 360 == 90:
            target_lat_rad, target_long_rad = CoordCalculations.calc_east_coord(curr_lat_rad, curr_long_rad, distance)
        elif bearing % 360 == 180:
            target_lat_rad, target_long_rad = CoordCalculations.calc_south_coord(curr_lat_rad, curr_long_rad, distance)
        elif bearing % 360 == 270:
            target_lat_rad, target_long_rad = CoordCalculations.calc_west_coord(curr_lat_rad, curr_long_rad, distance)
        else:
            return {}

        # convert latitude and longitude back to degrees
        target_lat = math.degrees(target_lat_rad)
        target_long = math.degrees(target_long_rad)
        coord = {'lat': target_lat, 'long': target_long, 'distance': distance, 'bearing': bearing % 360}

        return coord

    @staticmethod
    def calc_north_coord(curr_lat_rad, curr_long_rad, distance):
        target_lat_rad = math.asin(math.sin(curr_lat_rad) * math.cos(distance / CoordCalculations.earth_radius) + math.cos(curr_lat_rad) * math.sin(distance / CoordCalculations.earth_radius) * 1)

        return target_lat_rad, curr_long_rad

    @staticmethod
    def calc_east_coord(curr_lat_rad, curr_long_rad, distance):
        target_long_rad = curr_long_rad + math.atan2(1 * math.sin(distance / CoordCalculations.earth_radius) * math.cos(curr_lat_rad), math.cos(distance / CoordCalculations.earth_radius) - math.sin(curr_lat_rad) ** 2)

        return curr_lat_rad, target_long_rad

    @staticmethod
    def calc_south_coord(curr_lat_rad, curr_long_rad, distance):
        target_lat_rad = math.asin(math.sin(curr_lat_rad) * math.cos(distance / CoordCalculations.earth_radius) + math.cos(curr_lat_rad) * math.sin(distance / CoordCalculations.earth_radius) * -1)

        return target_lat_rad, curr_long_rad


    @staticmethod
    def calc_west_coord(curr_lat_rad, curr_long_rad, distance):
        target_long_rad = curr_long_rad + math.atan2(-1 * math.sin(distance / CoordCalculations.earth_radius) * math.cos(curr_lat_rad), math.cos(distance / CoordCalculations.earth_radius) - math.sin(curr_lat_rad) ** 2)

        return curr_lat_rad, target_long_rad