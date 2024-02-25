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
        distance = 0
        final = 0
        for i in range(8, ((StateTimer.round_up_to_multiple_of_4(radius)) * 4) + 1, 8):
            distance = distance + i
            final = i
        print(distance)
        print(final)
        final1 = final + 8
        print(final1)
        distance1 = distance + final1/2
        print(distance1)
        totalTime = distance1/speed
        print(f"State Timer is {totalTime}")


    @staticmethod
    def test_timer():
        StateTimer.calculateTime(20)

# # Test the timer
# StateTimer.testTimer()
