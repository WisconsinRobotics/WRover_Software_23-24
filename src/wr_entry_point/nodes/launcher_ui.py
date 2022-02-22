#!/usr/bin/env python3

import os
import sys

import roslaunch
import roslaunch.parent
import roslaunch.rlutil
from rqt_gui.main import Main

from wr_entry_point import launcher_ui

def main():
    rqt_main = Main(filename='wr_entry_point')
    sys.exit(rqt_main.main(standalone='wr_entry_point'))

if __name__ == '__main__':
    main()
