#!/usr/bin/env python3
import rospy
import std_msgs.msg as std_msgs

class ClawController:
    OPEN_SPEED = 32767
    CLOSE_SPEED = -32768

    def __init__(self, topicName: str):
        self._open_pressed = False
        self._close_pressed = False
        self._publisher = rospy.Publisher(topicName, std_msgs.Int16)

    def open_claw(self, open):
        self._open_pressed = open
        self._publish_claw_speed()

    def close_claw(self, close):
        self._close_pressed = close
        self._publish_claw_speed()

    def _publish_claw_speed(self):
        speed = std_msgs.Int16(ClawController.OPEN_SPEED if self._open_pressed else ClawController.CLOSE_SPEED if self._close_pressed else 0)
        self._publisher.publish(speed)

def actuateSolenoid(msg: std_msgs.Bool):
    with open("/sys/class/gpio/gpio6/value", "w") as gpioFile:
        gpioFile.write("1" if msg.data else "0")
        gpioFile.flush()

if __name__ == "__main__":

    rospy.init_node("end_effector_controller")

    claw = ClawController("/hsi/roboclaw/aux3/cmd/left")
    openSubscriber = rospy.Subscriber("/hci/arm/gamepad/button/a", std_msgs.Bool, lambda msg: claw.open_claw(msg.data))
    closeSubscriber = rospy.Subscriber("/hci/arm/gamepad/button/b", std_msgs.Bool, lambda msg: claw.open_claw(msg.data))

    rospy.loginfo("Starting GPIO setup...")

    with open("/sys/class/gpio/export", "w") as exportFile:
        exportFile.write("6")
    with open("/sys/class/gpio/gpio6/direction", "w") as gpioFile:
        gpioFile.write("out")

    rospy.loginfo("GPIO setup complete!")
    solenoidSubscriber = rospy.Subscriber("/hci/arm/gamepad/button/y", std_msgs.Bool, actuateSolenoid, queue_size=1)

    rospy.spin()
    
    with open("/sys/class/gpio/unexport", "w") as unexportFile:
        unexportFile.write("6")