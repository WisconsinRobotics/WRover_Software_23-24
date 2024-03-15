class StateTimer:
    '''
    @ingroup wr_logic_ai
    @defgroup wr_logic_ai State Timer
    @brief Calculates the maximum travel time between all coordinates to know when to restart searching

    Methods(s):
      calculate_time(distance) - Calculates maximum travel time between all coordinates
    '''

    @staticmethod
    def calculate_time(radius = 20):
        '''
        Description: Calculates the maximum time it should take the rover to visit all 
            coordinates during the searching state. The spiral pattern uses 4 meters as a base,
            so the distance between every coordinate must be a multiple of 4.

        Parameter(s):
            radius - the radius of the square spiral, 
                correlates to the maximum distance the starting coordinate is to the target

        Return(s):
            time - the time it takes the rover to visit all coordinates
        '''
        speed = 0.5 # purposefully slow since true speed is inknown
                    # and gives more time for obstacle avoidance

        total_distance = (0.5 * radius + 1) * (2 * radius + 4)
        total_time = total_distance / speed

        return total_time
