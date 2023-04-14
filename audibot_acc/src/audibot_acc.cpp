#include <ros/ros.h> // ROS header file
#include <gazebo_msgs/ModelStates.h> // Gazebo messages to get positions of the models in the simulation
#include <cmath> // Math library for sqrt function used in displacement calculation
#include <geometry_msgs/Vector3.h> // pose.position is of Vector3 type (x, y, z [Cartesian])
#include <geometry_msgs/Twist.h> // Library for linear and angular velocity for x, y, and z axes.// Implementing header files in the program
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

// Program to implement adaptive cruise control ACC feature
// Version 1.2 
// Date: 4/14/2023
// Cleaned up code/comments and fixed errors in previous version
// Authors (github names):
// jseablom (jseablom)
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

int stage = 1;
//set 'stage' variable to:
// 0 - Level 1: Implement core system with ideal measurements
// 1 - Level 2: Use a LIDAR sensor to detect the lead vehicle
// 2 - Level 3: Use the camera to detect the lead vehicle

geometry_msgs::Pose target_vehicle_position;
geometry_msgs::Pose ego_vehicle_position; 
geometry_msgs::Twist twist_cmd;
// Set the desired following distance for adaptive cruise control
const double following_distance = 5.0; 
double twist_cmd_placeholder = 0;
// The value of '1' may need to be 'tuned' based on vehicle dynamics (braking/acceleration ability of audibot)

//Create static publisher
static ros::Publisher pub; 
static ros::Subscriber sub_laser;

double displacement = 100;

// Main function - Initializing the adaptive cruise control (ACC) node, subscribers and timers.


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

// Callback function whenever a new lidar scan message is received
void lidarCallbackFunction(const sensor_msgs::LaserScan::ConstPtr& msg) {
/*Header header            # timestamp in the header is the acquisition time of 
                         # the first ray in the scan.
                         #
                         # in frame frame_id, angles are measured around 
                         # the positive Z axis (counterclockwise, if Z is up)
                         # with zero angle being forward along the x axis
                         
float32 angle_min        # start angle of the scan [rad]
float32 angle_max        # end angle of the scan [rad]
float32 angle_increment  # angular distance between measurements [rad]

float32 time_increment   # time between measurements [seconds] - if your scanner
                         # is moving, this will be used in interpolating position
                         # of 3d points
float32 scan_time        # time between scans [seconds]

float32 range_min        # minimum range value [m]
float32 range_max        # maximum range value [m]

float32[] ranges         # range data [m] (Note: values < range_min or > range_max should be discarded)
float32[] intensities    # intensity data [device-specific units].  If your
                         # device does not provide intensities, please leave
                         # the array empty.*/
  displacement = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  ROS_INFO("Lidar message received...");


}

void twistTime(const geometry_msgs::TwistStamped msg) {
  twist_cmd_placeholder = msg.twist.angular.z;
}

// Timer callback - runs every 100ms
void timerCallback(const ros::TimerEvent& event) 
{
  if (stage == 0) {
  // Calculate x, y, and z displacement in cartesian coordinates and find geometric mean 
  double dx = target_vehicle_position.position.x - ego_vehicle_position.position.x;
  double dy = target_vehicle_position.position.y - ego_vehicle_position.position.y;
  double dz = target_vehicle_position.position.z - ego_vehicle_position.position.z;

  // The displacement calculatation is assuming both vehicles traveling in a straight line!
  // This is not true when the vehicles switch lanes or take a curve

  displacement = std::sqrt(dx*dx + dy*dy + dz*dz);
  }

  if (stage == 1) {

    if (displacement == std::numeric_limits<double>::infinity()) {
      ROS_INFO("Waiting for LIDAR scan message to be received...");
    return;
  }
  
}

  // Print calculated displacement to the console for debugging purposes
  ROS_INFO("Following distance = %f meters", displacement);

  // Control algorithm for linear speed based on straight-line displacement and defined following distance
  double linear_speed = 30.0; 

  // Try to get parameter "speed" from launch file
  if (!ros::param::get("speed", linear_speed)) {
    // If parameter not found, assign the default value + 0.1 (for debugging purposes)
    linear_speed = 13.1;
  }

  ROS_INFO("Steering angle = %f radians", twist_cmd_placeholder);
  ROS_INFO("Stage = %f + 1", stage);

  // Basic control algorithm, may want to implement PID controller, as Yaswanth was suggesting...
  // We may also want to control our speed based on time-to-collision (or 'TTC') or relative velocity.

  // If the distance is greater than the set following distance, 
  // maintain the same (set) linear speed, 
  // otherwise adjust the set speed based on the distance between the two vehicles.
  if (displacement > 2*following_distance) {
   linear_speed = displacement / (2*following_distance) * linear_speed; // Speed up - Open road
  } else if (displacement > following_distance && displacement <= 2*following_distance) {
   linear_speed = 13.1; // Traffic nearby - Maintain (default) speed 
  } else if (displacement > following_distance/2 && displacement <= following_distance) {
   linear_speed = displacement / following_distance * linear_speed; // Getttingc closer- Slow down slightly
  } else {
   linear_speed = 0; // Too close - Slow down quickly
  }

  ROS_INFO("Set linear speed = %f", linear_speed); // For debugging: print linear speed value

  // Publishing the speed command 
 
  // Note: Need to remap the twist messages from audibot_path_following node to this (acc) node 
  twist_cmd.linear.x = linear_speed; 
  //twist_cmd.angular.z = 0.0; // Not the correct angular.z value, need to pass from audibot_path_following node
  twist_cmd.angular.z = twist_cmd_placeholder;
  //Publish to twist_cmd 

  pub.publish(twist_cmd);
  //Was using "static ros::Publisher pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel", 1);" previously

}

int main(int argc, char **argv) {
  ros::init(argc, argv, "audibot_acc");

  if (stage == 1) { 
  double displacement = std::numeric_limits<double>::infinity(); // initialize to infinity
  }

  if (stage == 0) {
  double displacement = 0;
  }

  // Ceate a nodehandle for the acc node
  ros::NodeHandle node_handle;

  if (stage == 0) {
  // Subscriber to "gazebo/model_states"
  ros::Subscriber model_states_subscriber = node_handle.subscribe("/gazebo/model_states", 1, modelStatesCallbackFunction);
  }

  if (stage == 1) {
  // Subscriber to "/ego_vehicle/laser/scan"
  ros::Subscriber lidar_subscriber = node_handle.subscribe("/ego_vehicle/laser/scan", 1, lidarCallbackFunction);  
  ROS_INFO("Subscribed to stage 1...");
  }

  ros::Subscriber twist_subscriber = node_handle.subscribe("/ego_vehicle/twist", 1, twistTime);

  // Creat a timer for our node, currently set to 10 Hz (Argument specified in seconds)
  ros::Timer timer = node_handle.createTimer(ros::Duration(0.1), timerCallback);

  // Creates a cmd_vel publisher
  //pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel2", 1);
  pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel", 1);

  // Keep the node running
  ros::spin();

  //return 0;
}