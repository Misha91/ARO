#include <cmath>  // For M_PI.

#include "ros/ros.h"
#include "tf/tf.h"
#include "robot/robot.h"

using namespace std;
/* threshold - robot waypoint tolerance in meters */
Robot::Robot() :
  threshold_(0.1), go_backwards_(false) {
}

void Robot::odometryCallback (const nav_msgs::Odometry::ConstPtr& msg) {
  ROS_DEBUG("got odometry");
    robot_pose_ = msg->pose.pose;
    std_msgs::Int32 state;
    state.data = plan_.size();
    state_publisher_->publish(state);
  if (execute_) {
    followTheCarrot();
  }
}

void Robot::startExecutePath() {
  execute_start_time_ = ros::Time::now();
  execute_ = true;
}

void Robot::stopExecutePath() {
  geometry_msgs::Twist command;
  command.angular.z = 0;
  command.linear.x = 0;
  command_publisher_->publish(command);
  execute_ = false;
}

bool Robot::addPathService(robot_coordination::AddPath::Request& req, robot_coordination::AddPath::Response& res) {
    ROS_INFO("service addPath called");
    plan_.clear();
    ROS_INFO_STREAM("Plan has " << req.waypoints.size() << " points");
    for (int i =0; i < req.waypoints.size(); i++) {
        plan_.push_back(Waypoint(req.waypoints[i].pose,req.waypoints[i].timepoint));
        ROS_DEBUG_STREAM("added waypoint (" <<
            req.waypoints[i].pose.position.x <<
            ", " <<
            req.waypoints[i].pose.position.y <<
            ")");
    }
    res.result = true;
    std_msgs::Int32 state;
    state.data = plan_.size();
    state_publisher_->publish(state);
    return true;
}

bool Robot::startMovementService(robot_coordination::StartMovement::Request& req, robot_coordination::StartMovement::Response& res) {
    go_backwards_ = req.go_backwards;
    startExecutePath();
    res.ack = true;
    return true;
}

bool Robot::stopMovementService(robot_coordination::StopMovement::Request& req, robot_coordination::StopMovement::Response& res) {
   stopExecutePath();
   res.ack = true;
   return true;
}

void Robot::setPath(const std::list<Waypoint>& path) {
  plan_ = path;
}

void Robot::setCommandPublisher(ros::Publisher * cmdpub) {
  command_publisher_ = cmdpub;
}

void Robot::setStatePublisher(ros::Publisher * statepub) {
  state_publisher_ = statepub;
}

template <typename T> T sgn(T val) {
    return (T(0) < val) - (val < T(0));
}
bool Robot::followTheCarrot() {
  if (plan_.empty()) {
    geometry_msgs::Twist command;
    command.angular.z = 0;
    command.linear.x = 0;
    command_publisher_->publish(command);
    return true;
  } else {
    while (!plan_.empty() && plan_.front().distanceTo(robot_pose_) < threshold_) {
      plan_.pop_front();
    }
    if(plan_.empty())
        return true;
    Waypoint actual_goal = plan_.front();
    const double dist = actual_goal.distanceTo(robot_pose_);
    const double angle = actual_goal.angleTo(robot_pose_);
    tf::Quaternion q(
        robot_pose_.orientation.x,
        robot_pose_.orientation.y,
        robot_pose_.orientation.z,
        robot_pose_.orientation.w);
    tf::Matrix3x3 m(q);
    double roll;
    double pitch;
    double yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("distance %f and angle %f to goal", dist,angle);
    ROS_INFO("robot roll %f , pitch %f yaw %f", roll, pitch, yaw);
    geometry_msgs::Twist command;
    double delta = angle - yaw;
    if(go_backwards_)
        delta -= M_PI;
    if (delta > M_PI ) {
      delta -= 2 * M_PI;
    }
    if (delta < -M_PI ) {
      delta += 2 * M_PI;
    }
    ROS_INFO_STREAM(ros::Time::now() << " " << actual_goal.timepoint);
    const double time_to_goal = ((execute_start_time_ + actual_goal.timepoint) - ros::Time::now()).toSec();

    ROS_INFO("time to goal %f ", time_to_goal);
    double speed;
    double speed_limit = 0.2;
    if (time_to_goal > 0) {
      speed = dist/time_to_goal;
      if(speed>speed_limit)
        speed = speed_limit;
    } else {
     speed = speed_limit;
    }
    /* command.angular.z = 2 * (sin(delta) - 0.333 * sin(3 * delta)); */
    if(abs(delta) > 1) {
      command.angular.z = 1*sgn(delta);
    } else {
   
      command.angular.z = delta;
    }
    command.linear.x = speed * (cos(delta) + 0.333 * cos(3 * delta));
    if (command.linear.x < 0) {
      command.linear.x = 0;
    }
    if(go_backwards_)
        command.linear.x = - command.linear.x;
    command_publisher_->publish(command);

    return false;
  }
}
