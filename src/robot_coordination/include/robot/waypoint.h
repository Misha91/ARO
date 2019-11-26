#include "geometry_msgs/Pose.h"
#include "ros/ros.h"
class Waypoint {
  public:
  
    geometry_msgs::Pose pose; //< position of the waypoint
    ros::Duration  timepoint; //< time, when the waypoint should be reached, defined as duration from the execution start
    Waypoint();
    Waypoint (geometry_msgs::Pose, ros::Duration);
    double distanceTo(geometry_msgs::Pose);
    double angleTo(geometry_msgs::Pose);

};
