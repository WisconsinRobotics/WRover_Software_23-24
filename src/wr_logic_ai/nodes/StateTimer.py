import math

class StateTimer:

    @staticmethod
    def round_up_to_multiple_of_4(number):
        rounded_up = math.ceil(number)

        if rounded_up % 4 != 0:
            rounded_up += 4 - (rounded_up % 4)

        return rounded_up

    @staticmethod
    def calculate_time(radius = 20):
        speed = 0.5
        total_distance = 0
        last_path = 0

        for i in range(8, ((StateTimer.round_up_to_multiple_of_4(radius)) * 4) + 1, 8):
            total_distance += i
            last_path = i

        last_path += 8
        total_distance += last_path / 2
        total_time = total_distance / speed

        return total_time


    # @staticmethod
    # def test_timer():
    #     StateTimer.calculateTime(20)

# # Test the timer
# StateTimer.testTimer()
