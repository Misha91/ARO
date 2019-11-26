# Path following using the robot_coordination package

## Package content
- src/node.cpp - The main ROS node code.
- src/robot.cpp - Follow-the-carrot path following in C++.
- src/waypoint.cpp - Definition of the Waypoint data type and related routines.
- include/robot/*.h - Related C++ header files. 
- msg/Waypoint.msg - Waypoint message definition.
- srv/*.srv - Definitions of services, see below in ROS interfaces.
- scripts/path_following_simple.py - An example ROS node and a Python class showing how to follow a path from Python.

## ROS interfaces
- Waypoint message
    - geometry_msgs/Pose pose - The pose to go to. Note: only the X and Y coordinates are used.
    - duration timepoint - Time when the waypoint should be reached. Measured from the moment the service start_movement is called.  
- services
    - add_path 
        - request: Waypoint[] waypoints
        - response: bool result
    - start_movement
        - request: bool go_backwards
            - True - follow the path backwards
            - False - follow the path forwards
        - response: bool ack
    - stop_movement
        - request: 
        - response: bool ack
- topic waypoints_ahead, type Int32
    - msg.data - the number of waypoints left to follow, 0 when finished following the last trajectory

## Running
Start the main C++ ROS node:
```
rosrun robot_coordination robot_node
```
A waypoint navigation example. Follow a predefined path forwards:
```
rosrun robot_coordination path_following_simple.py
```
