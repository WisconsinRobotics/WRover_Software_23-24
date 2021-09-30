#!/usr/bin/env python
import rospy
import json

'''

This code is used to test the ROS Dashboard. It publishes fake data to the topics specified in the JSON file config.json


'''


#helper function
# -converts a string containing a class name into a python class
def get_class( kls ):
    parts = kls.split('.')
    module = ".".join(parts[:-1])
    m = __import__( module )
    for comp in parts[1:]:
        m = getattr(m, comp)
    return m


#helper function
# -converta a ROS class string name (e.g. "sensor_msgs/BatteryState") into an instantiable python class (sensor_msgs.msg._BatteryState.BatteryState)
def json_string_to_ros_class(json_pretty_string):
    split = json_pretty_string.split("/")
    ros_class_name = split[0] + ".msg._"+ split[1] +"." + split[1]
    return get_class(ros_class_name)

#load the json file containing user-editable ROS topics/types
with open('config.json') as json_file:
    visualization_descriptions = json.load(json_file)
    # buttons_descriptions = visualization_descriptions["buttons"]
    widgets_descriptions = visualization_descriptions["visualizations"]

# publish fake ROS messages to the appropriate topics/type specified in the JSON file. Runs at 10Hz
def publish_dummy_data():
    pub1 = rospy.Publisher(widgets_descriptions["fast_rate_small_circular_gauge_1"]["topic_name"], json_string_to_ros_class(widgets_descriptions["fast_rate_small_circular_gauge_1"]["ros_message_type"]), queue_size=10)
    pub2 = rospy.Publisher(widgets_descriptions["fast_rate_small_circular_gauge_2"]["topic_name"], json_string_to_ros_class(widgets_descriptions["fast_rate_small_circular_gauge_2"]["ros_message_type"]), queue_size=10)
    pub3 = rospy.Publisher(widgets_descriptions["fast_rate_small_circular_gauge_3"]["topic_name"], json_string_to_ros_class(widgets_descriptions["fast_rate_small_circular_gauge_3"]["ros_message_type"]), queue_size=10)
    pub4 = rospy.Publisher(widgets_descriptions["fast_rate_large_circular_gauge_1"]["topic_name"], json_string_to_ros_class(widgets_descriptions["fast_rate_large_circular_gauge_1"]["ros_message_type"]), queue_size=10)
    pub5 = rospy.Publisher(widgets_descriptions["fast_rate_large_circular_gauge_2"]["topic_name"], json_string_to_ros_class(widgets_descriptions["fast_rate_large_circular_gauge_2"]["ros_message_type"]), queue_size=10)
    pub6 = rospy.Publisher(widgets_descriptions["medium_rate_thick_meter_1"]["topic_name"], json_string_to_ros_class(widgets_descriptions["medium_rate_thick_meter_1"]["ros_message_type"]), queue_size=10)
    pub7 = rospy.Publisher(widgets_descriptions["medium_rate_thick_meter_2"]["topic_name"], json_string_to_ros_class(widgets_descriptions["medium_rate_thick_meter_2"]["ros_message_type"]), queue_size=10)
    pub8 = rospy.Publisher(widgets_descriptions["medium_rate_thick_meter_3"]["topic_name"], json_string_to_ros_class(widgets_descriptions["medium_rate_thick_meter_3"]["ros_message_type"]), queue_size=10)

    rospy.init_node('ros_dashboard_dummy_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz

    #get ros msg class name from json and instantiate a new msg of that type
    msg1 = json_string_to_ros_class(widgets_descriptions["fast_rate_small_circular_gauge_1"]["ros_message_type"])()
    msg2 = json_string_to_ros_class(widgets_descriptions["fast_rate_small_circular_gauge_2"]["ros_message_type"])()
    msg3 = json_string_to_ros_class(widgets_descriptions["fast_rate_small_circular_gauge_3"]["ros_message_type"])()
    msg4 = json_string_to_ros_class(widgets_descriptions["fast_rate_large_circular_gauge_1"]["ros_message_type"])()
    msg5 = json_string_to_ros_class(widgets_descriptions["fast_rate_large_circular_gauge_2"]["ros_message_type"])()
    msg6 = json_string_to_ros_class(widgets_descriptions["medium_rate_thick_meter_1"]["ros_message_type"])()
    msg7 = json_string_to_ros_class(widgets_descriptions["medium_rate_thick_meter_2"]["ros_message_type"])()
    msg8 = json_string_to_ros_class(widgets_descriptions["medium_rate_thick_meter_3"]["ros_message_type"])()

    #initialize the dummy values to the minimums specified in the json file
    msg1_value = widgets_descriptions["fast_rate_small_circular_gauge_1"]["mininum_value"]
    msg2_value = widgets_descriptions["fast_rate_small_circular_gauge_2"]["mininum_value"]
    msg3_value = widgets_descriptions["fast_rate_small_circular_gauge_3"]["mininum_value"]
    msg4_value = widgets_descriptions["fast_rate_large_circular_gauge_1"]["mininum_value"]
    msg5_value = widgets_descriptions["fast_rate_large_circular_gauge_2"]["mininum_value"]
    msg6_value = widgets_descriptions["medium_rate_thick_meter_1"]["mininum_value"]
    msg7_value = widgets_descriptions["medium_rate_thick_meter_2"]["mininum_value"]
    msg8_value = widgets_descriptions["medium_rate_thick_meter_3"]["mininum_value"]


    while not rospy.is_shutdown():

        #increment dummy values to cause the dashboard visualizations to move
        msg1_value+=1
        msg2_value+=1
        msg3_value+=1
        msg4_value+=1
        msg5_value+=1
        msg6_value+=1
        msg7_value+=1
        msg8_value+=1

        #cap the dummy value to the maximums specified in the json file
        msg1_final_val = msg1_value%widgets_descriptions["fast_rate_small_circular_gauge_1"]["maximum_value"]
        msg2_final_val = msg2_value%widgets_descriptions["fast_rate_small_circular_gauge_2"]["maximum_value"]
        msg3_final_val = msg3_value%widgets_descriptions["fast_rate_small_circular_gauge_3"]["maximum_value"]
        msg4_final_val = msg4_value%widgets_descriptions["fast_rate_large_circular_gauge_1"]["maximum_value"]
        msg5_final_val = msg5_value%widgets_descriptions["fast_rate_large_circular_gauge_2"]["maximum_value"]
        msg6_final_val = msg6_value%widgets_descriptions["medium_rate_thick_meter_1"]["maximum_value"]
        msg7_final_val = msg7_value%widgets_descriptions["medium_rate_thick_meter_2"]["maximum_value"]
        msg8_final_val = msg8_value%widgets_descriptions["medium_rate_thick_meter_3"]["maximum_value"]

        #set the ros message fields to dummy data.  We grab the message field name from json and user setattr to set it (capped at the max value in the json)
        #  --this is done because we don't know the ROS message type nor the ROS message field until runtime
        setattr(msg1, "data" if "std_msgs" in widgets_descriptions["fast_rate_small_circular_gauge_1"]["ros_message_type"] else widgets_descriptions["fast_rate_small_circular_gauge_1"]["field_name"], msg1_final_val)
        setattr(msg2, "data" if "std_msgs" in widgets_descriptions["fast_rate_small_circular_gauge_2"]["ros_message_type"] else widgets_descriptions["fast_rate_small_circular_gauge_2"]["field_name"] , msg2_final_val)
        setattr(msg3, "data" if "std_msgs" in widgets_descriptions["fast_rate_small_circular_gauge_3"]["ros_message_type"] else widgets_descriptions["fast_rate_small_circular_gauge_3"]["field_name"], msg3_final_val)
        setattr(msg4, "data" if "std_msgs" in widgets_descriptions["fast_rate_large_circular_gauge_1"]["ros_message_type"] else widgets_descriptions["fast_rate_large_circular_gauge_1"]["field_name"], msg4_final_val)
        setattr(msg5, "data" if "std_msgs" in widgets_descriptions["fast_rate_large_circular_gauge_2"]["ros_message_type"] else widgets_descriptions["fast_rate_large_circular_gauge_2"]["field_name"], msg5_final_val)
        setattr(msg6, "data" if "std_msgs" in widgets_descriptions["medium_rate_thick_meter_1"]["ros_message_type"] else widgets_descriptions["medium_rate_thick_meter_1"]["field_name"], msg6_final_val)
        setattr(msg7, "data" if "std_msgs" in widgets_descriptions["medium_rate_thick_meter_2"]["ros_message_type"] else widgets_descriptions["medium_rate_thick_meter_2"]["field_name"], msg7_final_val)
        setattr(msg8, "data" if "std_msgs" in widgets_descriptions["medium_rate_thick_meter_3"]["ros_message_type"] else widgets_descriptions["medium_rate_thick_meter_3"]["field_name"], msg8_final_val)

        #blast out the dummy data
        pub1.publish(msg1)
        pub2.publish(msg2)
        pub3.publish(msg3)
        pub4.publish(msg4)
        pub5.publish(msg5)
        pub6.publish(msg6)
        pub7.publish(msg7)
        pub8.publish(msg8)

        #debug output to compare against the dashboard values
        hello_str = "%i %i %i %i %i %i %i %i" %  (msg1_final_val,msg2_final_val,msg3_final_val,msg4_final_val,msg5_final_val,msg6_final_val,msg7_final_val,msg8_final_val)
        rospy.loginfo(hello_str)

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_dummy_data()
    except rospy.ROSInterruptException:
        pass