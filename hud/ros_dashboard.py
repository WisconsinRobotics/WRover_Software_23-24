#!/usr/bin/env python

import dash
import dash_core_components as dcc
import dash_html_components as html
import dash_daq as daq
import dash_bootstrap_components as dbc
import rospy
from dash.dependencies import Input, Output, State
from multiprocessing import Process, Manager
import json
from std_srvs.srv import Trigger, TriggerRequest

'''

This code creates a ROS Dashboard that listens to ROS topics/types specified in the JSON file config.json.

'''



#create data structures for sharing variables between ROS process and Dash process
manager = Manager()
shared_dict = manager.dict()
visualization_descriptions = manager.dict()

#initialize shared data structure to zeroes
shared_dict["GAUGE_1"] = 0
shared_dict["GAUGE_2"] = 0
shared_dict["GAUGE_3"] = 0
shared_dict["GAUGE_4"] = 0
shared_dict["GAUGE_5"] = 0
shared_dict["GAUGE_6"] = 0
shared_dict["GAUGE_7"] = 0
shared_dict["GAUGE_8"] = 0

#global service clients for button pushes (send a ROS service TriggeRequest upon button push)
button_1_trigger_service_client = None
button_2_trigger_service_client = None
button_3_trigger_service_client = None

#Initialize Dash app
external_stylesheets = ['https://codepen.io/chriddyp/pen/bWLwgP.css']
app = dash.Dash(external_stylesheets=[dbc.themes.BOOTSTRAP], title='ROS Dashboard')


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

# Fast rate callback for the fast gauges (small and large)
#   -takes the ROS data as input (now in json format) and writes it as each gauge's value
#   -implemented as a javascript clientside callback to increase speed on SBC devices (raspberry pi)
app.clientside_callback(
    """
    function(json_data) {
        var obj = JSON.parse(json_data);
        if (obj) {
            return [parseInt(obj.GAUGE_1), parseInt(obj.GAUGE_2), parseInt(obj.GAUGE_3),parseInt(obj.GAUGE_4),parseInt(obj.GAUGE_5)];
        }
        else {
            return [0,0,0,0,0];
        }
    }
    """,
    [
    Output('progress-gauge1', 'value'),
    Output('progress-gauge2', 'value'),
    Output('progress-gauge3', 'value'),
    Output('darktheme-daq-gauge1', 'value'),
    Output('darktheme-daq-gauge2', 'value'),
    ],
    [
    Input('intermediate-value', 'children'),
    ]
)


# Medium rate callback for the thick, meter gauges
#   -takes the ROS data as input (now in json format) and writes it as each meter's value
#   -implemented as a javascript clientside callback to increase speed on SBC devices (raspberry pi)
app.clientside_callback(
    """
    function(n_intervals, json_data) {
        var obj = JSON.parse(json_data);
        if (obj) {
            return [parseInt(obj.GAUGE_6), parseInt(obj.GAUGE_7), parseInt(obj.GAUGE_8)];
        }
        else {
            return [0,0,0];
        }
    }
    """,
    [
    Output('darktheme-daq-tank1', 'value'),
    Output('darktheme-daq-tank2', 'value'),
    Output('darktheme-daq-tank3', 'value'),
    ],
    Input('interval-component-medium', 'n_intervals'),
    State('intermediate-value', 'children')
)


# callback that dispatches button clicks and sends a trigger request to the corresponding ROS service mapped to a button
@app.callback(
    Output("button-click-sinkhole", "children"),
    [Input("button_1", "n_clicks"),
     Input("button_2", "n_clicks"),
     Input("button_3", "n_clicks")]
)
def on_button_click(btn1, btn2, btn3):
    global button_1_trigger_service_client
    global button_2_trigger_service_client
    global button_3_trigger_service_client

    ctx = dash.callback_context

    if not ctx.triggered:
        print("initialized button...")
    else:
        button_id = ctx.triggered[0]['prop_id'].split('.')[0]
        print("button clicked", button_id)

        button_trigger_request = TriggerRequest()
        try:
            if button_id == "button_1":
                response = button_1_trigger_service_client(button_trigger_request)
            elif button_id == "button_2":
                response = button_2_trigger_service_client(button_trigger_request)
            elif button_id == "button_3":
                response = button_3_trigger_service_client(button_trigger_request)
            else:
                response = None
                print("Couldn't map this button_id to a known button name: ", button_id)
            print("service response: ", response)
        except rospy.ServiceException as exc:
            print("Service did not process request: " + str(exc))
        return None


#Fast callback to take the ROS data dictionary and store it in json format (
#  -the json format is needed for the clientside javascript callbacks that do the fast clientside drawing of the widgets
@app.callback(
    Output('intermediate-value', 'children'),
    Input('interval-component-fast', 'n_intervals')
)
def fast_ros_dict_to_json( n_intervals ):
    return json.dumps( dict(shared_dict))


def callback1(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["ros_message_type"]):
        shared_dict["GAUGE_1"] = data.data
    #Ok, it's not a std_msgs.  Grab data from the message field described in the json file
    else:
        shared_dict["GAUGE_1"] = getattr(data, visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["field_name"])
def callback2(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["ros_message_type"]):
        shared_dict["GAUGE_2"] = data.data
    else:
        # Ok, it's not a std_msgs.  Grab data from the message field described in the json file
        shared_dict["GAUGE_2"] = getattr(data, visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["field_name"])
def callback3(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["ros_message_type"]):
        shared_dict["GAUGE_3"] = data.data
    else:
        # Ok, it's not a std_msgs.  Grab data from the message field described in the json file
        shared_dict["GAUGE_3"] = getattr(data, visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["field_name"])
def callback4(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["ros_message_type"]):
        shared_dict["GAUGE_4"] = data.data
    else:
        # Ok, it's not a std_msgs.  Grab data from the message field described in the json file
        shared_dict["GAUGE_4"] = getattr(data, visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["field_name"])
def callback5(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["ros_message_type"]):
        shared_dict["GAUGE_5"] = data.data
    else:
        shared_dict["GAUGE_5"] = getattr(data, visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["field_name"])
def callback6(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["ros_message_type"]):
        shared_dict["GAUGE_6"] = data.data
    else:
        # Ok, it's not a std_msgs.  Grab data from the message field described in the json file
        shared_dict["GAUGE_6"] = getattr(data, visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["field_name"])
def callback7(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["ros_message_type"]):
        shared_dict["GAUGE_7"] = data.data
    else:
        # Ok, it's not a std_msgs.  Grab data from the message field described in the json file
        shared_dict["GAUGE_7"] = getattr(data, visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["field_name"])
def callback8(data):
    #if the ROS message is a std_msgs, then the payload is always in the .data field.
    if ("std_msgs" in visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["ros_message_type"]):
        shared_dict["GAUGE_8"] = data.data
    else:
        # Ok, it's not a std_msgs.  Grab data from the message field described in the json file
        shared_dict["GAUGE_8"] = getattr(data, visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["field_name"])


# initialize subscribers to feed each visualization widget on the dashboard
def init_sub():
    rospy.init_node('ros_dashboard')

    #A subscriber for each visualization.
    # -the topic name is grabbed from the json file
    # -the ROS message type is grabbed from the json file as a string and interpreted to the respective Python class
    rospy.Subscriber(visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["ros_message_type"]), callback1)
    rospy.Subscriber(visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["ros_message_type"]), callback2)
    rospy.Subscriber(visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["ros_message_type"]), callback3)
    rospy.Subscriber(visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["ros_message_type"]), callback4)
    rospy.Subscriber(visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["ros_message_type"]), callback5)
    rospy.Subscriber(visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["ros_message_type"]), callback6)
    rospy.Subscriber(visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["ros_message_type"]), callback7)
    rospy.Subscriber(visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["topic_name"], json_string_to_ros_class(visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["ros_message_type"]), callback8)

    #spin forever (dash spinner is running in a separate process already)
    rospy.spin()


#initialize a ROS service client for each button inside the Dash process
def init_button_service_clients():
    global button_1_trigger_service_client
    global button_2_trigger_service_client
    global button_3_trigger_service_client

    button_1_trigger_service_client = rospy.ServiceProxy(visualization_descriptions["buttons"]["button_1"]["trigger_service_to_invoke"], Trigger)
    button_2_trigger_service_client = rospy.ServiceProxy(visualization_descriptions["buttons"]["button_2"]["trigger_service_to_invoke"], Trigger)
    button_3_trigger_service_client = rospy.ServiceProxy(visualization_descriptions["buttons"]["button_3"]["trigger_service_to_invoke"], Trigger)


#Setup the layout of the dashboard.  3 buttons, 3 small fast gauges, 2 large fast gauges, and 3 thick slow meters
def setup_dash_app():

    theme = {
        'dark': True,
        'detail': '#007439',
        'primary': '#00EA64',
        'secondary': '#6E6E6E',
    }

    row = html.Div(
        [
            #BUTTON ROW
            dbc.Row(
                [
                    #BUTTON 1 (IF ENABLED IN JSON FILE)
                    dbc.Col(dbc.Button(visualization_descriptions["buttons"]["button_1"]["visualization_description"], block=True, id="button_1", color="success", size="lg", className="mr-1"), width=3) \
                        if visualization_descriptions["buttons"]["button_1"]["enabled"] else None,
                    #BUTTON 2 (IF ENABLED IN JSON FILE)
                    dbc.Col(dbc.Button(visualization_descriptions["buttons"]["button_2"]["visualization_description"], block=True,  id="button_2", color="danger", size="lg", className="mr-1"), width=3) \
                        if visualization_descriptions["buttons"]["button_2"]["enabled"] else None,
                    #BUTTON 3 (IF ENABLED IN JSON FILE)
                    dbc.Col(dbc.Button(visualization_descriptions["buttons"]["button_3"]["visualization_description"], block=True, id="button_3", color="info", size="lg", className="mr-1"), width=3) \
                        if visualization_descriptions["buttons"]["button_3"]["enabled"] else None,
                ],
                justify="center",
                align="center",
            ),

            html.Hr(),

            #SMALL GAUGE ROW
            dbc.Row(
                [
                    #FAST SMALL CIRCLE GAUGE 1 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Gauge(
                        id="progress-gauge1",
                        min=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["maximum_value"],
                        showCurrentValue=True,
                        label = {'label': visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["visualization_label"],
                                 'style': {'font-size': '25px',"color": "#8dd7f9"}},
                        units=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["visualization_units"],
                        size=120, ), )
                        if visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_1"]["enabled"] else None,
                    #FAST SMALL CIRCLE GAUGE 2 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Gauge(
                        id="progress-gauge2",
                        min=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["maximum_value"],
                        showCurrentValue=True,
                        label = {'label': visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["visualization_label"],
                                 'style': {'font-size': '25px',"color": "#8dd7f9"}},
                        units=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["visualization_units"],
                        size=120, ), )
                        if visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_2"]["enabled"] else None,
                    # FAST SMALL CIRCLE GAUGE 3 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Gauge(
                        id="progress-gauge3",
                        min=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["maximum_value"],
                        showCurrentValue=True,
                        label = {'label': visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["visualization_label"],
                                 'style': {'font-size': '25px',"color": "#8dd7f9"}},
                        units=visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["visualization_units"],
                        size=120, ), )
                        if visualization_descriptions["visualizations"]["fast_rate_small_circular_gauge_3"]["enabled"] else None,
                ],
                align="center",

            ),

            #LARGE GAUGE ROW
            dbc.Row(
                [
                    # FAST LARGE CIRCLE GAUGE 1 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Gauge(
                        color=theme['primary'],
                        id='darktheme-daq-gauge1',
                        className='dark-theme-control',
                        min=visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["maximum_value"],
                        showCurrentValue=True,  # default size 200 pixel
                        label={
                            'label': visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["visualization_label"],
                            'style': {'font-size': '25px', "color": "#00EA64"}},
                        units=visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["visualization_units"],
                        ), )
                        if visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_1"]["enabled"] else None,
                    # FAST LARGE CIRCLE GAUGE 1 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Gauge(
                        color=theme['primary'],
                        id='darktheme-daq-gauge2',
                        className='dark-theme-control',
                        min=visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["maximum_value"],
                        showCurrentValue=True,
                        label={
                            'label': visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["visualization_label"],
                            'style': {'font-size': '25px', "color": "#00EA64"}},
                        units=visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["visualization_units"],
                    ), )
                    if visualization_descriptions["visualizations"]["fast_rate_large_circular_gauge_2"]["enabled"] else None,

                ],
                align="center",
            ),

            #THICK METER ROW
            dbc.Row(
                [
                    # SLOW THICK METER 1 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Tank(
                        id='darktheme-daq-tank1',
                        className='dark-theme-control',
                        style={'margin-left': '18px'},
                        min=visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["maximum_value"],
                        showCurrentValue=True,
                        label={
                            'label': visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"][
                                "visualization_label"],
                            'style': {'font-size': '25px', "color": "#00EA64"}},
                            units=visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"][
                            "visualization_units"],
                        ), )
                        if visualization_descriptions["visualizations"]["medium_rate_thick_meter_1"]["enabled"] else None,
                    # SLOW THICK METER 2 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Tank(
                        id='darktheme-daq-tank2',
                        className='dark-theme-control',
                        style={'margin-left': '18px'},
                        min=visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["maximum_value"],
                        showCurrentValue=True,
                        label={
                            'label': visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["visualization_label"],
                            'style': {'font-size': '25px', "color": "#00EA64"}},
                            units=visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["visualization_units"],
                        ), )
                        if visualization_descriptions["visualizations"]["medium_rate_thick_meter_2"]["enabled"] else None,
                    # SLOW THICK METER 3 (IF ENABLED IN JSON FILE)
                    dbc.Col(daq.Tank(
                        id='darktheme-daq-tank3',
                        className='dark-theme-control',
                        style={'margin-left': '18px'},
                        min=visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["mininum_value"],
                        max=visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["maximum_value"],
                        showCurrentValue=True,
                        label={
                            'label': visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["visualization_label"],
                            'style': {'font-size': '25px', "color": "#00EA64"}},
                            units=visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["visualization_units"],
                        ), )
                        if visualization_descriptions["visualizations"]["medium_rate_thick_meter_3"]["enabled"] else None,
                ],
                align="center",
            ),

            # Hidden div inside the app that stores the intermediate value.  This is used for clientside callback
            html.Div(id='intermediate-value', style={'display': 'none'}),

            # Hidden div that just acts as a sinkhole.  Each callback is required to have an output
            html.Div(id='button-click-sinkhole', style={'display': 'none'}),
        ]
    )


    app.layout = dbc.Container(children=[
        # two callback timers that operate at different speeds
        daq.DarkThemeProvider(
            theme=theme, children=[row,
                                   dcc.Interval(
                                       id='interval-component-fast',
                                       interval=250,  # in milliseconds
                                       n_intervals=0
                                   ),
                                   dcc.Interval(
                                       id='interval-component-medium',
                                       interval=5000,  # in milliseconds
                                       n_intervals=0
                                   ),
                                   ])
    ],
        style={'backgroundColor': '#303030',
               'margin-top': '25px' },
    )


if __name__ == '__main__':
    print("Starting ROS Dashboard")

    # global visualization_descriptions
    with open('config.json') as json_file:
      visualization_descriptions = json.load(json_file)

    #initialize dashboard layout
    setup_dash_app()

    #spawn a process to run all the ROS listeners.  Used a thread safe dictionary to transfer data from the ROS process
    # to the Dash process
    p = Process(target=init_sub, args=())
    p.start()

    #the dash process requires some ROS publishers (service clients really) to execute button commands
    init_button_service_clients()

    #spin the Dash server
    app.run_server(host="0.0.0.0", debug=False, port=8050,  threaded=True)
