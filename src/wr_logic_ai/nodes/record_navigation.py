import rospy
import signal
import sys
import rosbag
from drive.msg import Drive
import time

FILE_NAME = 'drive_commands.bag'
topic_name = '/drive_cmd'
msg_list = []

def write_data():
    try:
        print('Creating bag file')
        bag = rosbag.Bag(FILE_NAME, 'w')
        for msg in msg_list:
            bag.write(topic_name, msg) 
    finally:
        bag.close()

def record_data(data):
    data.left = -1 * (data.left)
    data.right = -1 * (data.right)
    print(data)
    time1 = time.time()
    msg_list.append(data)
    time2 = time.time()
    print(time2 - time1)
 
def signal_handler(sig, frame):
    print('\nScript stopped by user')
    write_data()
    sys.exit(0)

def initialize():
    rospy.init_node('navigation_recorder', anonymous=False)
    
    #Subscribers
    rospy.Subscriber(topic_name, Drive, record_data)
    print('Recording data from ' + topic_name)
    rospy.spin()

signal.signal(signal.SIGINT, signal_handler)
signal.signal(signal.SIGTERM, signal_handler)
initialize()

