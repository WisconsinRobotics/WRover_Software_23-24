#!/usr/bin/env python3

from typing import Any, Generic, List, Optional, TypeVar, cast

import rospy
from std_msgs.msg import Bool, Float32, Int16
import time

TURNTABLE_SPEED_FACTOR = 0.2
SHOULDER_SPEED_FACTOR = 0.4
ELBOW_SPEED_FACTOR = 0.3
FOREARM_SPEED = 16384
# WRIST_SPEED = 12288
WRIST_SPEED = 14746

class Watchdog:
    def __init__(self, timeout: float):
        self._timeout = timeout
        self.pet()

    def pet(self):
        self._lastTime = time.clock_gettime(0)

    def isMad(self):
        return time.clock_gettime(0) - self._lastTime >= self._timeout

T = TypeVar('T')
w = Watchdog(1)

class SubBuf(Generic[T]):
    def __init__(self, topic_name: str, topic_type: type):
        self.data: Optional[T] = None
        self.sub = rospy.Subscriber(topic_name, topic_type, self._msg_cb)

    def _msg_cb(self, msg: Any):
        self.data = cast(T, msg.data)
        w.pet()

def float_to_int16_msg(value: float, factor: float = 1.0) -> Int16:
    factor = min(abs(factor), 1.0)
    return Int16(round(factor * value * 32767))


def main():
    rospy.init_node('bad_arm_driver')

    # retrieve params
    controller_ns = cast(str, rospy.get_param('~controller_ns'))
    claw_ns_0 = cast(str, rospy.get_param('~claw_ns_0'))
    claw_ns_1 = cast(str, rospy.get_param('~claw_ns_1'))
    claw_ns_2 = cast(str, rospy.get_param('~claw_ns_2'))
    claw_ns_3 = cast(str, rospy.get_param('~claw_ns_3'))
    spin_rate = rospy.get_param('~spin_rate', 50)

    # create controller subs
    trigger_l: SubBuf[float] = SubBuf(f'{controller_ns}/axis/trigger_left', Float32)
    trigger_r: SubBuf[float] = SubBuf(f'{controller_ns}/axis/trigger_right', Float32)
    stick_l: SubBuf[float] = SubBuf(f'{controller_ns}/axis/stick_left_y', Float32)
    stick_r: SubBuf[float] = SubBuf(f'{controller_ns}/axis/stick_right_y', Float32)
    bumper_l: SubBuf[bool] = SubBuf(f'{controller_ns}/button/shoulder_l', Bool)
    bumper_r: SubBuf[bool] = SubBuf(f'{controller_ns}/button/shoulder_r', Bool)
    pov_x: SubBuf[float] = SubBuf(f'{controller_ns}/axis/pov_x', Float32)
    pov_y: SubBuf[float] = SubBuf(f'{controller_ns}/axis/pov_y', Float32)
    #btn_a: SubBuf[bool] = SubBuf(f'{controller_ns}/button/a', Bool)
    #btn_b: SubBuf[bool] = SubBuf(f'{controller_ns}/button/b', Bool)
    btn_x: SubBuf[bool] = SubBuf(f'{controller_ns}/button/x', Bool)
    #btn_y: SubBuf[bool] = SubBuf(f'{controller_ns}/button/y', Bool)

    # create wroboclaw pubs
    pub_turntable = rospy.Publisher(f'{claw_ns_0}/cmd/left', Int16, queue_size=4)
    pub_shoulder = rospy.Publisher(f'{claw_ns_0}/cmd/right', Int16, queue_size=4)
    pub_elbow = rospy.Publisher(f'{claw_ns_1}/cmd/left', Int16, queue_size=4)
    pub_forearm = rospy.Publisher(f'{claw_ns_1}/cmd/right', Int16, queue_size=4)
    pub_wrist_a = rospy.Publisher(f'{claw_ns_2}/cmd/left', Int16, queue_size=4)
    pub_wrist_b = rospy.Publisher(f'{claw_ns_2}/cmd/right', Int16, queue_size=4)
    # pub_eef = rospy.Publisher(f'{claw_ns_3}/cmd/left', Int16, queue_size=4)

    # main loop
    sleeper = rospy.Rate(spin_rate)
    while not rospy.is_shutdown():
        if not w.isMad():
            shoulder_max_speed = SHOULDER_SPEED_FACTOR
            elbow_max_speed = ELBOW_SPEED_FACTOR

            # Speed control for arm shoulder and elbow joints
            if btn_x.data is not None and btn_x.data:
                shoulder_max_speed = 1
                elbow_max_speed = 1

            if trigger_l.data is not None and trigger_r.data is not None:
                pub_turntable.publish(float_to_int16_msg(trigger_r.data - trigger_l.data, TURNTABLE_SPEED_FACTOR))
            
            if stick_l.data is not None:
                pub_shoulder.publish(float_to_int16_msg(stick_l.data, shoulder_max_speed))
            
            if stick_r.data is not None:
                pub_elbow.publish(float_to_int16_msg(stick_r.data, elbow_max_speed))
            
            # Bumpers used for cam mast control
            if bumper_l.data is not None and bumper_r.data is not None:
                if bumper_l.data:
                    if bumper_r.data:
                        pub_forearm.publish(Int16(0))
                    pub_forearm.publish(Int16(-FOREARM_SPEED))
                elif bumper_r.data:
                    pub_forearm.publish(Int16(FOREARM_SPEED))
                else:
                    pub_forearm.publish(Int16(0))
            
            if pov_x.data is not None and pov_y.data is not None:
                wrist_spd_a = 0
                wrist_spd_b = 0
                if pov_x.data > 0:
                    wrist_spd_a = -1
                    wrist_spd_b = -1
                elif pov_x.data < 0:
                    wrist_spd_a = 1
                    wrist_spd_b = 1
                elif pov_y.data > 0:
                    wrist_spd_a = 1
                    wrist_spd_b = -1
                elif pov_y.data < 0:
                    wrist_spd_a = -1
                    wrist_spd_b = 1
                pub_wrist_a.publish(Int16(WRIST_SPEED * wrist_spd_a))
                pub_wrist_b.publish(Int16(-WRIST_SPEED * wrist_spd_b))

            # End effector is handled by end_effector_controller.py

            # if btn_a.data is not None and btn_b.data is not None:
            #     if btn_a.data:
            #         if btn_b.data:
            #             pub_eef.publish(Int16(0))
            #         pub_eef.publish(Int16(24576))
            #     elif btn_b.data:
            #         pub_eef.publish(Int16(-24576))
            #     else:
            #         pub_eef.publish(Int16(0))
        else:
            pub_turntable.publish(Int16(0))
            pub_shoulder.publish(Int16(0))
            pub_elbow.publish(Int16(0))
            pub_forearm.publish(Int16(0))
            pub_wrist_a.publish(Int16(0))
            pub_wrist_b.publish(Int16(0))
            # pub_eef.publish(Int16(0))
        sleeper.sleep()

if __name__ == '__main__':
    main()
