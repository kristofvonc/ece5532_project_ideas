#include <ros/ros.h> // ROS header file
#include <gazebo_msgs/ModelStates.h> // Gazebo messages to get positions of the models in the simulation
#include <cmath> // Math library for sqrt function used in displacement calculation
#include <geometry_msgs/Vector3.h> // pose.position is of Vector3 type (x, y, z [Cartesian])
#include <geometry_msgs/Twist.h> // Library for linear and angular velocity for x, y, and z axes.// Implementing header files in the program
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>



int main(int argc, char **argv) {
  ros::init(argc, argv, "audibot_acc");

  // Ceate a nodehandle for the acc node
  ros::NodeHandle node_handle;


  // Keep the node running
  ros::spin();

  //return 0;
}