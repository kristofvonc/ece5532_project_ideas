//implementing header files in the program
#include <ros/ros.h> //ros header file
#include <std_msgs/Float32.h> //for debugging purposes, not relevant for ACC implementation
#include <gazebo_msgs/ModelState.h> //gazebo messages to get positions of the models in the simulation
#include <cmath> //math library for sqrt function used in displacment calculation
#include <geometry_msgs/Vector3.h> //pose.position us if Vector3 type (x,y,z)
#include <geometry_msgs/Twist.h> //library for linear and angular velocity for x,y,z, 
//in the context of audibot, we use the x component for linear velocity and y component for angular velocity 

//Level 1: Implement core system with ideal measurements
//Look up the states of the two cars' Gazebo models, extract their positions, and compute the exact distance between them.
//Use the computed distance as input to an algorithm that controls the speed of the following car such that it maintains a relative distance to the lead vehicle.
//The audibot_path_following node publishes to a geometry_msgs/Twist topic to hold a constant speed and to follow the lane markings. To impose a different speed for ACC, remap this topic to the ACC node and
//then replace the linear.x field with a new speed, keeping the angular.z unchanged

//stores the position of the target and ego vehicle models
geometry_msgs::Pose target_vehicle_position;
geometry_msgs::Pose ego_vehicle_position; 

//set the desired following distance for adaptive cruise control
const double following_distance = 1.0; 
//value of '1' may need to be 'tuned' based on vehicle dynamics (braking/acceleration ability of audibot)

//callback function whenever a new /gazebo/model_states message is received
void modelStatesCallbackFunction(const gazebo_msgs::ModelState::ConstPtr& msg) {
  //rostopic echo /gazebo/model_states gives:
  //name: 
  //- ground_plane (index 0)
  //- lane_merge_0 (index 1)
  //- lane_merge_2 (index 2)
  //- lane_merge_3 (index 3)
  //- lane_merge_4 (index 4)
  //- ego_vehicle (index 5)
  //- target_vehicle (index 6)
  //end of topic /gazebo/model_states name list

  //assign values based on index (above)
  ego_vehicle_position = msg->pose[5]; 
  target_vehicle_position = msg->pose[6]; 
}

void timerCallback(const ros::TimerEvent& event) 
{
  //calculate x,y,z displacment in cartesian coordinates and find geometric mean (RMS)
  double dx = target_vehicle_position.position.x - ego_vehicle_position.position.x;
  double dy = target_vehicle_position.position.y - ego_vehicle_position.position.y;
  double dz = target_vehicle_position.position.z - ego_vehicle_position.position.z;
  double displacement  = std::sqrt(dx*dx + dy*dy + dz*dz);

  //print calculated displacment to the console - debugging purposes
  ROS_INFO("Displacement = %f", displacement);

  //control algorithm for linear speed based on displacment and set following distance
  double linear_speed; 

  //try to get parameter "speed" from launch file
  if (!ros::param::get("speed", linear_speed)) {
    //if parameter not found, assign the default value + 0.1 (for debugging purposes)
    linear_speed = 23.1;
  }

  //basic control algorithm, may want to implement PID controller, as yaswanth was suggesting 
  if (displacement > following_distance) {
    linear_speed = 23;
  } else { 
    linear_speed = displacement/following_distance
  }

  ROS_INFO("Linear speed = %f", linear_speed); //for debugging

  //publishing the speed command
  geometry_msgs::Twist twist_cmd;
  //NOT IMPLEMENTED: need to remap the twist messages from audibot_path_following node to this (acc) node 
  twist_cmd.linear.x = linear_speed; 
  twist_cmd.angular.z = 0.0; //not the correct angular.z value, need to pass from audibot_path_following node
  twist_cmd_publisher.publish(twist_cmd)

  //for debugging - not relevant to ACC implementation
  acc_msg.data = 420.69;
  acc_publisher.publish(acc_msg);
}

//initializing the adaptive cruise control (ACC) node 
//int main(int argc, char** argv) {
int main(int argc, char **argv) {
  ros::init(argc, argv, "acc_node");

  //used to access specific ROS parameters
  //ros::NodeHandle node_handle("~"); 

  //create a nodehandle for the acc node
  ros::NodeHandle node_handle;

  //creating an object in the /ego_vehicle namespace
  ros::NodeHandle node_handle_namespace(node_handle, "/ego_vehicle"); 

  //publisher for our node
  //below publisher for debugging only, not relevant to ACC implementation
  ros::Publisher acc_publisher = node_handle_namespace.advertise<std_msgs::Float32>("acc", 1);
  //publisher for the twist_cmd
  ros::Publisher twist_cmd_publisher = node_handle.advertise<geometry_msgs::("twist_cmd", 1);

  //subscriber to gazebo/model_states
  ros::Subscriber model_states_subscriber = node_handle_namespace.subscribe("/gazebo/model_states", 1, modelStatesCallbackFunction);

  //publishing some data to the'acc' topic within the 'ego_vehicle' namespace
  std_msgs::Float32 acc_msg;

  //for timing our node, currently set to 20 Hz (argument specified in seconds)
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.05), timerCallback);

  //keep the node running
  ros::spin();
}