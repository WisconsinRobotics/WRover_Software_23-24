#!/usr/bin/env python3

import os
from typing import Callable

import rospy

def assert_envvar(var_name: str, value_pred: Callable[[str], bool], value_err_fn: Callable[[str], str]):
    var_value = os.environ.get(var_name)
    if not var_value:
        raise ValueError(f'Expected environment variable "{var_name}", but it does not exist!')
    if not value_pred(var_value):
        raise ValueError(value_err_fn(var_value))

def assert_envvar_vals(var_name: str, *permissible_vals: str):
    permissible_val_set = set(permissible_vals)
    def get_err_str(value: str):
        p_val_names = ', '.join(f'"{value}"' for value in permissible_vals)
        return f'Expected environment variable "{var_name}" to be one of {{{p_val_names}}}, but got "{value}"!'
    assert_envvar(var_name, lambda v: v in permissible_val_set, get_err_str)

def main():
    rospy.init_node('env_checker')

    try:
        assert_envvar_vals('WROVER_LOCAL', 'true', 'false')
        assert_envvar_vals('WROVER_HW', 'REAL', 'MOCK')
    except ValueError as e:
        rospy.logerr(str(e))
        raise

    # just idle, since this node will be required by the launch
    sleeper = rospy.Rate(1) # use this instead of spin to consume less cpu time
    while not rospy.is_shutdown():
        sleeper.sleep()

if __name__ == '__main__':
    main()
