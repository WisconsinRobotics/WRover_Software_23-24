# wr_hci_hud

@defgroup wr_hci_hud wr_hci_hud
@brief A package (currently empty) to act as a HUD for drivers

# To start the GUI

Run test.py in wr_hci_hud/src  

run roscore
run roslaunch rosbridge_server rosbridge_websocket.launch

the ros.js file is currently publishing and then subscribing to the ros websocket to simulate actual input

# Video streaming
for webRTC Streaming, might require more steps. WebRTC isn't implemented fully yet.
Plan to use https://wiki.ros.org/web_video_server for video streaming for now


## History

Ideas for this node have gone through several iterations.  None of the following have been implemented far enough to have a good estimate for the efficacy of the method:

* A locally hosted webserver on the base station (probably easiest in Python) that live-updates its components to reflect subscriptions over ROS.
  * Pros:
    * Most of the GUI heavy-lifting is taken care of by the web browser
    * Lots of options/documentation for the web server itself
    * Could be accessed/used by any other computer on the network (good for debugging)
  * Cons:
    * May be hidden complexity to configure layout
    * Transporting images/geospatial data (if rendered) may take up a lot of bandwidth
    * Rendering geospatial data/proprioception might not be easy
    * Camera streaming might not be possible due to bandwidth, may still need extra tool
    * Lack of Internet at competition might limit design or force pre-downloaded dependencies
* Qt/Tk/similar locally hosted application that is ROS-aware.
  * Pros:
    * Lots of language options
    * Broad customizability
    * Likely easier ROS integration (due to customizability)
    * Easier to make visual rendering more efficient
  * Cons:
    * Analysis Paralysis
      * The team does more of the legwork on the GUI magic
      * Likely complex configuration
    * Constrained to platform compiled for, or need to distribute the binary
* Direct console access (current solution)
  * Pros:
    * Few/no layers of extra support required between operator and WRover
    * No GUI work
  * Cons:
    * Operator must be ***extremely*** proficient in ROS/`bash`/filesystems/networks
      * No assistance/recovery available if something goes wrong
    * Hard to visualize large number of subsystems at once
    * Slow to operate
    * Hard to construct complex diagnostic tools and insights on the fly
    * Little to no visual support outside of `rqt`

Given past experience at competition, even a small GUI with no input options (a literal HUD) would be massively beneficial compared to reading individaul diagnostics one at a time.  This could then be expanded to provide more detailed insights, inputs to the WRover, and specializations for the different competition modes.

