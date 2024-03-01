import math

class CoordCalculations:
    '''
    @ingroup wr_logic_ai
    @defgroup wr_logic_ai Coordinate Calculations
    @brief Calculates the coordinates the rover travels to when searching.

    Attributes(s):
        EARTH_RADIUS - radius of the Earth

    Methods(s):
        get_coords(start_lat, start_long, distance, num_vertices) - gets the list of coordinates
        calc_dest_coord(curr_lat, curr_long, distance, bearing) - calculates the target coordinate
        calc_north_coord(curr_lat_rad, curr_long_rad, distance) - calculates coordinate going north
        calc_east_coord(curr_lat_rad, curr_long_rad, distance) - calculates coordinate going east
        calc_south_coord(curr_lat_rad, curr_long_rad, distance) - calculates coordinate going south
        calc_west_coord(curr_lat_rad, curr_long_rad, distance) - calculates coordinate going west
    '''
    
    EARTH_RADIUS = 6378100 # in meters

    @staticmethod
    def get_coords(start_lat, start_long, distance, num_vertices):
        '''
        Description: 'Main' method that gets the coordinates the rover will travel to in the current 
            searching round based on the parameters. We only need to call this method.

        Parameter(s): 
            start_lat - the rover's starting latitude in the current searching round
            start_long - the rover's starting longitude in the current searching round
            distance - the distance traveled between the starting coordinate and the next 
                        immediate coordinate
            num_vertices - the maximum number of coordinates the rover could travel to

        Return(s):
            coords - list of coordinates the rover will travel to during the search state
        '''
        orig_dist = distance
        coords = []
        bearing = 0
        mult = 0

        # create starting coordinate as the first coordinate
        coords.append({'lat': start_lat, 'long': start_long})

        for i in range(num_vertices):
            # update the distance after it has been traveled twice to create the square spiral shape
            if (i > 1) and (i % 2) == 0:
                mult += orig_dist

            coords.append(CoordCalculations.calc_dest_coord(coords[i]['lat'], coords[i]['long'], 
                    distance + mult, bearing))
            bearing += 90

        return coords

    @staticmethod
    def calc_dest_coord(curr_lat, curr_long, distance, bearing):
        '''
        Description: Calls the respective cardinal direction method based on the bearing.

        Parameter(s):
            curr_lat - the current latitude in degrees
            curr_long - the current longitude in degrees
            distance - the distance that will be traveled
            bearing - the bearing/direction that will be traveled toward

        Return(s):
            coord - the newly calculated coordinate, containing:
                target_lat - the newly calculated target latitude in degrees
                target_long - the newly calculated target longitude in degrees
                (what it took to get to the target latitude and longitude)
                distance - the distance that was traveled
                bearing - the bearing/direction that was traveled toward
        '''
        target_lat_rad = 0
        target_long_rad = 0

         # convert current latitude and longitude from degrees to radians
        curr_lat_rad = math.radians(curr_lat)
        curr_long_rad = math.radians(curr_long)

        # call respective direction method
        if bearing % 360 == 0:
            target_lat_rad, target_long_rad = CoordCalculations.calc_north_coord(
                    curr_lat_rad, curr_long_rad, distance)
        elif bearing % 360 == 90:
            target_lat_rad, target_long_rad = CoordCalculations.calc_east_coord(
                    curr_lat_rad, curr_long_rad, distance)
        elif bearing % 360 == 180:
            target_lat_rad, target_long_rad = CoordCalculations.calc_south_coord(
                    curr_lat_rad, curr_long_rad, distance)
        elif bearing % 360 == 270:
            target_lat_rad, target_long_rad = CoordCalculations.calc_west_coord(
                    curr_lat_rad, curr_long_rad, distance)
        else:
            return {}

        # convert latitude and longitude back to degrees
        target_lat = math.degrees(target_lat_rad)
        target_long = math.degrees(target_long_rad)

        # create the new coordinate
        coord = {'lat': target_lat, 'long': target_long, 'distance': distance, 'bearing': bearing % 360}

        return coord
    
    # The next four methods are based on the formula 'Destination point given distance and bearing 
    # from start point' from https://www.movable-type.co.uk/scripts/latlong.html. Based on these 
    # formulas, north has a bearing of 0°, east has a bearing of 90°, south has a bearing of 180°, 
    # and west has a bearing of 270°. 
    #
    # With the variables curr_lat, curr_long, distance, bearing, and EARTH_RADIUS, 
    #   target_lat and target_long can be calculated using these formulas:
    #
    #       target_lat = asin(
    #           sin(curr_lat) 
    #           ⋅ cos(distance / EARTH_RADIUS) 
    #           + cos(curr_lat) 
    #           ⋅ sin(distance / EARTH_RADIUS) 
    #           ⋅ cos(bearing))
    #
    #       target_long = curr_long + atan2(
    #           sin(bearing) 
    #           ⋅ sin(distance / EARTH_RADIUS) 
    #           ⋅ cos(curr_lat), 
    #           cos(distance / EARTH_RADIUS) 
    #           − sin(curr_lat)^2))

    @staticmethod
    def calc_north_coord(curr_lat_rad, curr_long_rad, distance):
        '''
        Description: Calculates the target latitude based on the current latitude and distance 
            traveled north. Only latitude needs to be calculated since the longitude does not change 
            when going north.

        Parameter(s):
            curr_lat_rad - the current latitude in radians
            curr_long_rad - the current longitude in radians
            distance - the distance that will be traveled

        Return(s):
            target_lat_rad - the newly calculated target latitude in radians
            curr_long_rad - the same current longitude in radians
        '''
        target_lat_rad = math.asin(
                math.sin(curr_lat_rad) 
                * math.cos(distance / CoordCalculations.EARTH_RADIUS) 
                + math.cos(curr_lat_rad) 
                * math.sin(distance / CoordCalculations.EARTH_RADIUS) 
                * 1)

        return target_lat_rad, curr_long_rad

    @staticmethod
    def calc_east_coord(curr_lat_rad, curr_long_rad, distance):
        '''
        Description: Calculates the target longitude based on the current latitude, current longitude, 
            and distance traveled east. Only longitude needs to be calculated since the latitude 
            does not change when going east.

        Parameter(s):
            curr_lat_rad - the current latitude in radians
            curr_long_rad - the current longitude in radians
            distance - the distance that will be traveled

        Return(s):
            curr_lat_rad - the same current latitude in radians
            target_long_rad - the newly calculated target longitude in radians
        '''
        target_long_rad = curr_long_rad + math.atan2(
                1 
                * math.sin(distance / CoordCalculations.EARTH_RADIUS) 
                * math.cos(curr_lat_rad), 
                math.cos(distance / CoordCalculations.EARTH_RADIUS) 
                - math.sin(curr_lat_rad) ** 2)

        return curr_lat_rad, target_long_rad

    @staticmethod
    def calc_south_coord(curr_lat_rad, curr_long_rad, distance):
        '''
        Description: Calculates the target latitude based on the current latitude and distance 
            traveled south. Only latitude needs to be calculated since the longitude does not change 
            when going south.

        Parameter(s):
            curr_lat_rad - the current latitude in radians
            curr_long_rad - the current longitude in radians
            distance - the distance that will be traveled

        Return(s):
            target_lat_rad - the newly calculated target latitude in radians
            curr_long_rad - the same current longitude in radians
        '''
        target_lat_rad = math.asin(
                math.sin(curr_lat_rad) 
                * math.cos(distance / CoordCalculations.EARTH_RADIUS) 
                + math.cos(curr_lat_rad) 
                * math.sin(distance / CoordCalculations.EARTH_RADIUS) 
                * -1)

        return target_lat_rad, curr_long_rad

    @staticmethod
    def calc_west_coord(curr_lat_rad, curr_long_rad, distance):
        '''
        Description: Calculates the target longitude based on the current latitude, current longitude, 
            and distance traveled west. Only longitude needs to be calculated since the latitude 
            does not change when going west.

        Parameter(s):
            curr_lat_rad - the current latitude in radians
            curr_long_rad - the current longitude in radians
            distance - the distance that will be traveled

        Return(s):
            curr_lat_rad - the same current latitude in radians
            target_long_rad - the newly calculated target longitude in radians
        '''
        target_long_rad = curr_long_rad + math.atan2(
                -1 
                * math.sin(distance / CoordCalculations.EARTH_RADIUS) 
                * math.cos(curr_lat_rad), 
                math.cos(distance / CoordCalculations.EARTH_RADIUS) 
                - math.sin(curr_lat_rad) ** 2)

        return curr_lat_rad, target_long_rad