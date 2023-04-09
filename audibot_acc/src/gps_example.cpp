#include <ros/ros.h> // ROS header file
#include <gazebo_msgs/ModelStates.h> // Gazebo messages to get positions of the models in the simulation
#include <cmath> // Math library for sqrt function used in displacement calculation
#include <geometry_msgs/Vector3.h> // pose.position is of Vector3 type (x, y, z [Cartesian])
#include <geometry_msgs/Twist.h> // Library for linear and angular velocity for x, y, and z axes.// Implementing header files in the program
#include <geometry_msgs/Pose.h>

/*Program to implement adaptive cruise control ACC feature*/
// Version 1.1 
// Date: 4/2/2023
// Cleaned up code/comments and fixed errors in previous version
// Authors (github names):
// jseablom (N/A)
// Kacper Wojtowicz (KacoerWijtowicz)
// YawanthBoppana (rogueassassin14)
// Kristof von Czarnowski (kristofvonc)

// In the context of audibot, we use the x component for linear velocity and y component for angular velocity (steering)

//Level 1: Implement core system with ideal measurements
//Look up the states of the two cars' Gazebo models, extract their positions, and compute the exact distance between them.
//Use the computed distance as input to an algorithm that controls the speed of the following car such that it maintains a relative distance to the lead vehicle.
//The audibot_path_following node publishes to a geometry_msgs/Twist topic to hold a constant speed and to follow the lane markings. To impose a different speed for ACC, remap this topic to the ACC node and
//then replace the linear.x field with a new speed, keeping the angular.z unchanged

// Global variables to store the position of the target and ego vehicle models

geometry_msgs::Pose target_vehicle_position;
geometry_msgs::Pose ego_vehicle_position; 

// Set the desired following distance for adaptive cruise control
const double following_distance = 1.0; 
// The value of '1' may need to be 'tuned' based on vehicle dynamics (braking/acceleration ability of audibot)

//Create static publisher
static ros::Publisher pub; 

// Callback function whenever a new /gazebo/model_states message is received
void modelStatesCallbackFunction(const gazebo_msgs::ModelStates msg) {
  // "rostopic echo /gazebo/model_states" gives:

  // name: 
  // - ground_plane (index 0)
  // - lane_merge_0 (index 1)
  // - lane_merge_2 (index 2)
  // - lane_merge_3 (index 3)
  // - lane_merge_4 (index 4)
  // - ego_vehicle (index 5)
  // - target_vehicle (index 6)

  // End of topic /gazebo/model_states name list

  //Assign values based on index (above)
  //poses = msg->pose;
  //const auto& ego_vehicle_position = msg->pose[5].position; // Index 5 corresponds to EGO vehicle model           
  //const auto& target_vehicle_position = msg->pose.position; // Index 6 corresponds to Target vehicle model
  ego_vehicle_position  = msg.pose[5];
  target_vehicle_position = msg.pose[6];

}

// Timer callback - runs every 100ms
void timerCallback(const ros::TimerEvent& event) 
{
  // Calculate x, y, and z displacement in cartesian coordinates and find geometric mean 
  double dx = target_vehicle_position.position.x - ego_vehicle_position.position.x;
  double dy = target_vehicle_position.position.y - ego_vehicle_position.position.y;
  double dz = target_vehicle_position.position.z - ego_vehicle_position.position.z;

  // The displacement calculatation is assuming both vehicles traveling in a straight line!
  // This is not true when the vehicles switch lanes or take a curve

  double displacement  = std::sqrt(dx*dx + dy*dy + dz*dz);

  // Print calculated displacement to the console for debugging purposes
  ROS_INFO("Displacement = %f", displacement);

  // Control algorithm for linear speed based on straight-line displacement and defined following distance
  double linear_speed = 0.0; 

  // Try to get parameter "speed" from launch file
  if (!ros::param::get("speed", linear_speed)) {
    // If parameter not found, assign the default value + 0.1 (for debugging purposes)
    linear_speed = 23.1;
  }

  // Basic control algorithm, may want to implement PID controller, as Yaswanth was suggesting...
  // We may also want to control our speed based on time-to-collision (or 'TTC') or relative velocity.

  // If the distance is greater than the set following distance, 
  // maintain the same (set) linear speed, 
  // otherwise adjust the set speed based on the distance between the two vehicles.
  if (displacement > following_distance) {
    linear_speed = linear_speed; // Maintain speed - Do not speed up - Speeding ticket!
  } else { 
    linear_speed = displacement/following_distance * linear_speed; // Adjust speed based on distance
  }

  ROS_INFO("Linear speed = %f", linear_speed); // For debugging: print linear speed value

  // Publishing the speed command 
  geometry_msgs::Twist twist_cmd;
  // Note: Need to remap the twist messages from audibot_path_following node to this (acc) node 
  twist_cmd.linear.x = linear_speed; 
  twist_cmd.angular.z = 0.0; // Not the correct angular.z value, need to pass from audibot_path_following node
  //Publish to twist_cmd 
  pub.publish(twist_cmd);
  //Was using "static ros::Publisher pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel", 1);" previously

}

// Main function - Initializing the adaptive cruise control (ACC) node, subscribers and timers.
int main(int argc, char **argv) {
  ros::init(argc, argv, "audibot_acc_node");

  // Ceate a nodehandle for the acc node
  ros::NodeHandle node_handle;

  // Subscriber to "gazebo/model_states"
  ros::Subscriber model_states_subscriber = node_handle.subscribe("/gazebo/model_states", 1, modelStatesCallbackFunction);

  // Creat a timer for our node, currently set to 20 Hz (Argument specified in seconds)
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.05), timerCallback);

  // Creates a cmd_vel publisher
  pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel", 1);

  // Keep the node running
  ros::spin();

  //return 0;
}