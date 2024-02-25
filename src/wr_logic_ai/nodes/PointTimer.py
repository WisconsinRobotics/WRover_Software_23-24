class PointTimer:
  '''
    @ingroup wr_logic_ai
    @defgroup wr_logic_ai Point Timer
    @brief Calculates the maximum travel time between two coordinates to know when to skip the coordinate

    Methods(s):
      calculate_time(distance) - Calculates maximum travel time between two coordinates
    '''

  def calculate_time(distance):
    '''
    Description: Calculates the maximum time it should take the rover to travel from one 
      coordinate to another coordinate.

    Parameter(s):
      distance - the distance from the current coordinate to the target point

    Return(s):
      time - the time it takes the rover to travel the distance between two coordinates
    '''
    speed = 0.5 # purposefully slow since true speed is inknown 
                # and gives more time for obstacle avoidance
    time = distance / speed
    return time
