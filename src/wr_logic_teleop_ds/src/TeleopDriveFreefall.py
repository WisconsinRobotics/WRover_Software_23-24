import rospy
from wr_drive_msgs.msg import DriveTrainCmd
from wreadinput import DeviceAxis


left_drive_pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=10)
right_drive_pub = rospy.Publisher("/control/drive_system/cmd", DriveTrainCmd, queue_size=10)



def pub_drive_freefall():
   
    rate = rospy.Rate(10) # 10hz
    rospy.Subscriber("/hci/freefall/gamepad/stick_right_y", left_callback)
    rospy.Subscriber("/hci/freefall/gamepad/stick_left_y", right_callback)

def left_callback(msg):
    left_drive_pub.publish = -1*msg.data

def right_callback(msg):
    right_drive_pub.publish = -1*msg.data



if __name__ == '__main__':
    try:
         rospy.init_node('pub_drive_freefall', anonymous=True)
         pub_drive_freefall()
    except rospy.ROSInterruptException:
        pass