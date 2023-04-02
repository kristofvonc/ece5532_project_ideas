#include <ros/ros.h>

//Look up the states of the two cars' Gazebo models, extract their positions, and compute the exact distance between them.
//Use the computed distance as input to an algorithm that controls the speed of the following car such that it maintains a relative distance to the lead vehicle.
//The audibot_path_following node publishes to a geometry_msgs/Twist topic to hold a constant speed and to follow the lane markings. To impose a different speed for ACC, remap this topic to the ACC node and
//then replace the linear.x field with a new speed, keeping the angular.z unchanged.


//set up node

//define specific following distance 

//subscribe to gazbo_msgs/ModelStates messages

//extract distances of each vehicle by  extract the pose element for each

//in format of latitude and longitude ? x, y or z?

//calculate the geometric mean distance

//scale linear.x cmd_vel default speed based on that distance (angular.z unchanged)

//publish new topic 'cmd_vel2'

int main(int argc, char** argv) {
  ros::init(argc, argv, "tf_broadcaster");
  ros::NodeHandle node; 

  pub_marker_array = node.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

  ros::Timer timer = node.createTimer(ros::Duration(0.05), timerCallback);

  //subscribe to cmb vel topic
  //republish to cmd_vel2 topic
  //change to cmd_vel2 in launch file

  ros::spin();
}