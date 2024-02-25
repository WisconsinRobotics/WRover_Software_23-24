import math

class StateTimer:
    '''
    @ingroup wr_logic_ai
    @defgroup wr_logic_ai State Timer
    @brief Calculates the maximum travel time between all coordinates to know when to restart searching

    Methods(s):
      calculate_time(distance) - Calculates maximum travel time between all coordinates
    '''
      
    @staticmethod
    def round_up_to_multiple_of_4(number):
        '''
        Description: Rounds up a number to a multiple of 4.

        Parameter(s):
            number - the number to be rounded up to a multiple of 4

        Return(s):
            rounded_up - the rounded up multiple of 4
        '''
        rounded_up = math.ceil(number)

        if rounded_up % 4 != 0:
            rounded_up += 4 - (rounded_up % 4)

        return rounded_up

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
        total_distance = 0
        last_path = 0 # the spiral pattern has one extra path that is traveled to make 
                      # each spiral a full square

        # iterate through multiples of 4 to add to the total distance traveled in the searching state
        for i in range(8, ((StateTimer.round_up_to_multiple_of_4(radius)) * 4) + 1, 8):
            total_distance += i
            last_path = i # only set here for the very last iteration

        last_path += 8 # add 8 to make it the next path
        total_distance += last_path / 2 # only half is needed to create a full square
        total_time = total_distance / speed

        return total_time


    # @staticmethod
    # def test_timer():
    #     StateTimer.calculateTime(20)

# # Test the timer
# StateTimer.testTimer()
