#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "sensor_msgs/LaserScan.h"
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Joy.h>
#include <algorithm>

//Variables for laser
int c=0;
int laser_process_every_nth=1;

ros::Publisher pub_led;
ros::Publisher marker_pub;
ros::Publisher pub_vel;
ros::Subscriber sub_scan;

//Optional joystic controll
ros::Subscriber sub_vel_joy;
ros::Subscriber sub_joy;
int joystic = 0;


float robotRadius = 0.25;
float safetyDistance = robotRadius + 0.1;
float min = 0;
float max = 1;

//Object tracker parameters
//importance parameter for the distance of an object from the robot
float d_par = 0.3;
//max distance for proximity from previous position
float r_max = 1.5;
//importance parameter for the distance of an object from the previous position
float r_par = 1;

//initialize previous position
geometry_msgs::Point p_prev;
geometry_msgs::Point p_robot;


float distance(float x1, float y1, float x2, float y2){
return sqrt( (x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

float limit(float dist){
        return !(dist<r_max)?r_max:dist;
}

void goal_visualization(geometry_msgs::Point p){
  //ROS_ERROR("Goal visualization x: %f, y: %f ", p.x, p.y);
  if(p.x == 0 && p.y == 0)
    return;

    visualization_msgs::Marker object, line_strip;
    object.header.frame_id = line_strip.header.frame_id = "/camera_depth_optical_frame";
    object.header.stamp = line_strip.header.stamp = ros::Time::now();
    object.ns = line_strip.ns = "points_and_lines";
    object.action = line_strip.action = visualization_msgs::Marker::ADD;
    object.pose.orientation.w = line_strip.pose.orientation.w = 1.0;

    object.id = 0;
    line_strip.id = 1;

    object.type = visualization_msgs::Marker::CYLINDER;
    line_strip.type = visualization_msgs::Marker::LINE_STRIP;

    // POINTS markers use x and y scale for width/height respectively
    object.scale.x = 0.2;
    object.scale.y = 0.2;
    object.scale.z = 0.3;
    // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    line_strip.scale.x = 0.05;

    // Set colour (rd)
    object.color.r = 1.0;
    object.color.a = 1.0;

    // Set colour (blue)
    line_strip.color.b = 1.0;
    line_strip.color.a = 1.0;


    //Set the position of the object
    object.pose.position.x = p.x;
    object.pose.position.y = p.y;
    object.pose.position.z = 0.15;
    object.pose.orientation.x = 0.0;
    object.pose.orientation.y = 0.0;
    object.pose.orientation.z = 0.0;
    object.pose.orientation.w = 1.0;


    geometry_msgs::Point p0;
    p0.x = 0;
    p0.y = 0;
    p0.z = 0;

    //set up the line
    line_strip.points.push_back(p0);
    line_strip.points.push_back(p);

    marker_pub.publish(object);
    marker_pub.publish(line_strip);

}


 //LASER FUNCTION
void laserCallback(sensor_msgs::LaserScan msg) {
  c++;
  if (c < laser_process_every_nth)
    return;
  c = 0;

	if(joystic == 1)
		return;

  std::vector<float> vec;
  vec = msg.ranges;

//closest object
        float close = 10000;
        float best_dist = 10000;
        float best = 1000;
        int i_best =-1;
  geometry_msgs::Point best_object;
        best_object.x = 1;
        best_object.y = 0;

        for (int i =0; i<(vec.size());i++){


        float dev = msg.angle_min + msg.angle_increment * i;
  geometry_msgs::Point object;
        object.x = cos(dev) * vec.at(i);
        object.y = sin(dev) * vec.at(i);

        float lim = limit(10);

        float dist = distance(p_prev.x, p_prev.y, object.x,object.y);
        dist = limit(dist);

        float value = vec.at(i)*d_par + dist*r_par;

        if(vec.at(i) < safetyDistance){
                value = value/10;
        }

                if(value < best)
                {
                        best = value;
                        best_dist = vec.at(i);
                        best_object = object;
                        i_best = i;
                }
        }



float deviation = msg.angle_min + msg.angle_increment * i_best;

//ROS_ERROR(" ");
//ROS_ERROR("Goal angle: %f , value: %f ", deviation ,best);
//ROS_ERROR("Dist: %f , dist from prev: %f ", best_dist, distance(p_prev.x, p_prev.y, best_object.x,best_object.y));

p_prev = best_object;

if (best_dist>robotRadius)
{
  goal_visualization(best_object);
}

//ROS_ERROR("Something went wrong");

geometry_msgs::Twist cmd_vel;

float middle = (max + min)/2;


        cmd_vel.angular.z = 3*deviation/fabs(msg.angle_min);
        int led = 2;

        //object is either too close to the robot, or too far away
        if(best_dist > 4*max + 0.5 || best_dist < min)
        {
                cmd_vel.linear.x = 0;
                cmd_vel.angular.z =0;
                led =0;
        }
        //The distance is just right :)
        else if(best_dist > (middle - 0.1) && best_dist < (middle + 0.1))
        {
                cmd_vel.linear.x = 0;
        }
        //set the speed
        else
        {
                if(best_dist > max) best_dist = max;
                cmd_vel.linear.x = (best_dist - middle );
        }


std::cout << "pub vel: " << cmd_vel.linear.x << "\n";
//ROS_ERROR("Published velocity linear: %f, angular: %f", cmd_vel.linear.x, cmd_vel.angular.z);

float max_an = 1.5;
float max_lin = 0.4;

//saturation
if(cmd_vel.linear.x > max_lin) cmd_vel.linear.x = max_lin;
if(cmd_vel.linear.x < -max_lin) cmd_vel.linear.x = -max_lin;
if(cmd_vel.angular.z > max_an) cmd_vel.angular.z = max_an;
if(cmd_vel.angular.z < -max_an) cmd_vel.angular.z = -max_an;



	//pub_led.publish(led);
	pub_vel.publish(cmd_vel);
}




void joy_switch(sensor_msgs::Joy msg) {

	int button = msg.buttons.at(0);
	//ROS_ERROR("Button %d", button);

	if (button == 1) {
		if(joystic == 0){
			joystic = 1;
      p_prev.x = 0.7;
      p_prev.y = 0;
      p_prev.z = 0;
    }
		else
			joystic = 0;
	}

}

void velocity_joy(geometry_msgs::Twist cmd_vel) {

	if(joystic == 1)
		pub_vel.publish(cmd_vel);
}



int main(int argc, char** argv) {
  //init the ROS node
  ros::init(argc, argv, "follow");
  ros::NodeHandle n;

  std::cout << "follow\n";

  //laser subscriber
  sub_scan = n.subscribe("/scan", 1, laserCallback);

	//optional joystic control
	sub_joy = n.subscribe("/joy",1,joy_switch);
	sub_vel_joy = n.subscribe("/cmd_vel/joy",1,velocity_joy);


  pub_vel = n.advertise<geometry_msgs::Twist> ("/cmd_vel", 1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 0 );
  ros::Rate r(10);

  while(ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }


}
