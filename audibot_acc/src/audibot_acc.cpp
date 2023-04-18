#include <ros/ros.h> // ROS header file
#include <cmath> // Math library for sqrt function used in displacement calculation

//Basic ACC
#include <geometry_msgs/Twist.h> // Library for linear and angular velocity for x, y, and z axes.// Implementing header files in the program
#include <geometry_msgs/TwistStamped.h>

//Stage 1 libraries
#include <gazebo_msgs/ModelStates.h> // Gazebo messages to get positions of the models in the simulation
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h> // pose.position is of Vector3 type (x, y, z [Cartesian])

//Stage 2 libraries
#include <sensor_msgs/LaserScan.h>
#include <algorithm>

//Stage 3 libraries
#include <sensor_msgs/Image.h>

// jseablom (jseablom)
// Kacper Wojtowicz (KacoerWijtowicz)
// YaswanthBoppana (rogueassassin14)
// Kristof von Czarnowski (kristofvonc)

int stage = 1;
//set 'stage' variable to:
// 0 - Level 1: Implement core system with ideal measurements
// 1 - Level 2: Use a LIDAR sensor to detect the lead vehicle
// 2 - Level 3: Use the camera to detect the lead vehicle

// Set the desired following distance for adaptive cruise control
const double following_distance = 10.0; 

//Create geoemtry messages 
geometry_msgs::Pose target_vehicle_position;
geometry_msgs::Pose ego_vehicle_position;
geometry_msgs::Pose prev_ego_vehicle_position;
geometry_msgs::Twist target_vehicle_vel;
geometry_msgs::Twist ego_vehicle_vel; 
geometry_msgs::Twist twist_cmd;
geometry_msgs::Twist cmd_vel;
//Create static publishers
static ros::Publisher pub; 
static ros::Subscriber sub_laser;

double twist_cmd_placeholder = 0;
double displacement = 100;
double g_relative_vel = 0;
double g_displacement_rate = 0;
double g_ttc = 0;
//Placement of ego vehicle camera
//(5,-5,2) units in the X, Y, and Z directions, respectively
//(0.275643, 2.35619) represent the rotation around the X (roll) and Y (pitch) axes in radians
double camera_height = 2.0; // Height of the ego camera from the ground plane in meters
double camera_pitch = 0.275643; // Pitch angle of the ego camera in radians
double camera_fov = 1.3962634; // Field of view of the ego camera in radians
double lead_vehicle_height = 1.0; // Height of the lead vehicle from the ground plane in meters

//Create static publisher
// Callback function whenever a new /gazebo/model_states message is received
void modelStatesCallbackFunction(const gazebo_msgs::ModelStates msg) {
  ego_vehicle_position  = msg.pose[1];
  target_vehicle_position = msg.pose[2];

  ego_vehicle_vel  = msg.twist[1];
  target_vehicle_vel = msg.twist[2];

// Calculate relative velocity and displacement rate

  g_relative_vel = target_vehicle_vel.linear.x - ego_vehicle_vel.linear.x;
  double x_displacement = target_vehicle_position.position.x - ego_vehicle_position.position.x;
  double y_displacement = target_vehicle_position.position.y - ego_vehicle_position.position.y;
  g_displacement_rate = sqrt(x_displacement * x_displacement + y_displacement * y_displacement) / 1.0;

   // Calculate TTC using TTC algorithm
  double ttc = 0;
  if (g_relative_vel != 0)
  {
    ttc = g_displacement_rate / g_relative_vel;
  }

  g_ttc = ttc;

  if (stage == 0)
  {
    twist_cmd_placeholder = 0;
  }
  else if (stage == 1)
  {
    if (g_ttc <= 0)
    {
      twist_cmd_placeholder = 0;
    }
    else if (g_ttc > 0 && g_ttc <= 1)
    {
      twist_cmd_placeholder = 0.5;
    }
    else
    {
      twist_cmd_placeholder = 1;
    }
  }
  else if (stage == 2)
  {
    if (g_ttc <= 0)
    {
      twist_cmd_placeholder = 0;
    }
    else if (g_ttc > 0 && g_ttc <= 1)
    {
      twist_cmd_placeholder = 1;
    }
    else
    {
      twist_cmd_placeholder = 2;
    }
  }
  g_ttc = ttc;

  // Check if ego vehicle is too close to target vehicle
  if (g_displacement_rate < following_distance) {
    // Apply braking action
    twist_cmd.linear.x = 100; // Set linear velocity to 0
    twist_cmd.angular.z = 0; // Set angular velocity to 0
  } else {
    // Apply adaptive cruise control action
    twist_cmd.linear.x = ego_vehicle_vel.linear.x + twist_cmd_placeholder; // Set linear velocity to desired speed
    twist_cmd.angular.z = 0; // Set angular velocity to 0
  }

  // Publish the twist command
  pub.publish(twist_cmd);

}

// Callback function whenever a new lidar scan message is received
void lidarCallbackFunction(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (stage == 1) {
  displacement = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  //double linear_speed = 30.0; 
  }
}

// Callback function whenever a new camera image is received
void cameraCallbackFunction(const sensor_msgs::Image::ConstPtr& msg) {
    // Convert ROS image message to OpenCV image
    // Crop and resize image
    // Convert image to grayscale and apply Canny edge detection
    // Find contours in the image
    // Find the contour with the largest area, which corresponds to the lead vehicle
    // Calculate the distance to the lead vehicle using trigonometry
}

void twistTime(const geometry_msgs::TwistStamped msg) {
  twist_cmd_placeholder = msg.twist.angular.z;
}

// Timer callback - runs every 100ms
void timerCallback(const ros::TimerEvent& event) {
  if (stage == 0) {
    double dx = target_vehicle_position.position.x - ego_vehicle_position.position.x;
    double dy = target_vehicle_position.position.y - ego_vehicle_position.position.y;
    double dz = target_vehicle_position.position.z - ego_vehicle_position.position.z;
    displacement = std::sqrt(dx*dx + dy*dy + dz*dz);

    // Calculating rate of change of displacement using ego vehicle's position

    double dx_dt = (ego_vehicle_position.position.x - prev_ego_vehicle_position.position.x) / event.current_real.toSec();
    double dy_dt = (ego_vehicle_position.position.y - prev_ego_vehicle_position.position.y) / event.current_real.toSec();
    double dz_dt = (ego_vehicle_position.position.z - prev_ego_vehicle_position.position.z) / event.current_real.toSec();

    double displacement_rate = std::sqrt(dx_dt*dx_dt + dy_dt*dy_dt + dz_dt*dz_dt); 
    g_displacement_rate = displacement_rate;

    // Calculating relative velocity between Ego vehicle and Target vehicle

    double vx = target_vehicle_vel.linear.x - ego_vehicle_vel.linear.x;
    //double vy = target_vehicle_vel.linear.y - ego_vehicle_vel.linear.y;
    //double vz = target_vehicle_vel.linear.z - ego_vehicle_vel.linear.z;
    double relative_vel = abs(vx);
    g_relative_vel= relative_vel;
    prev_ego_vehicle_position = target_vehicle_position;
  } 
  else if (stage == 1) {
    if (displacement == std::numeric_limits<double>::infinity()) {
      ROS_INFO("Waiting for LIDAR scan message to be received...");
    }
    double dx = target_vehicle_position.position.x - ego_vehicle_position.position.x;
    double dy = target_vehicle_position.position.y - ego_vehicle_position.position.y;
    double dz = target_vehicle_position.position.z - ego_vehicle_position.position.z;
    displacement = std::sqrt(dx*dx + dy*dy + dz*dz);
    // Calculating rate of change of displacement using ego vehicle's position

    double dx_dt = (ego_vehicle_position.position.x - prev_ego_vehicle_position.position.x) / event.current_real.toSec();
    double dy_dt = (ego_vehicle_position.position.y - prev_ego_vehicle_position.position.y) / event.current_real.toSec();
    double dz_dt = (ego_vehicle_position.position.z - prev_ego_vehicle_position.position.z) / event.current_real.toSec();

    double displacement_rate = std::sqrt(dx_dt*dx_dt + dy_dt*dy_dt + dz_dt*dz_dt); 
    g_displacement_rate = displacement_rate;

    // Calculating relative velocity between Ego vehicle and Target vehicle

    double vx = target_vehicle_vel.linear.x - ego_vehicle_vel.linear.x;
    double relative_vel = std::abs(vx);
    g_relative_vel = relative_vel;
    if (relative_vel > 0) {
      double ttc = displacement / relative_vel;
      ROS_INFO_STREAM("Time-to-Collision (TTC): " << ttc << " seconds");
      g_ttc = ttc;
  }
  }
  if (stage == 2) {
    //PLACEHOLDER
    displacement = 100;
  }

  // Print calculated displacement to the console for debugging purposes
  ROS_INFO("Following distance = %f meters", displacement);

  // Control algorithm for linear speed based on straight-line displacement and defined following distance
  double linear_speed = 30.0; 

  // Try to get parameter "speed" from launch file
  if (!ros::param::get("speed", linear_speed)) {
    // If parameter not found, assign the default value + 0.1 (for debugging purposes)
  linear_speed = 120.0;
  }

  // Basic control algorithm, may want to implement PID controller, as Yaswanth was suggesting...
  // We may also want to control our speed based on time-to-collision (or 'TTC') or relative velocity.
  // trying to control the car using TTC
  
  //double ttc = g_displacement_rate / g_relative_vel; 
  if(g_ttc<8) {
    linear_speed = target_vehicle_vel.linear.x;
    }
      linear_speed = std::max(linear_speed, 120.0);
      ego_vehicle_vel.linear.x = linear_speed;
   //pub.publish(cmd_vel);
   
  /*if (displacement > 2*following_distance) {
   linear_speed = displacement / (2*following_distance) * linear_speed; // Speed up - Open road
  } else if (displacement > following_distance && displacement <= 2*following_distance) {
   linear_speed = 13.1; // Traffic nearby - Maintain (default) speed 
  } else if (displacement > following_distance/2 && displacement <= following_distance) {
   linear_speed = displacement / following_distance * linear_speed; // Getttingc closer- Slow down slightly
  } else {
   linear_speed = 0; // Too close - Slow down quickly
  } */

  ROS_INFO("Set linear speed = %f", linear_speed); // For debugging: print linear speed value
  twist_cmd.linear.x = linear_speed; 

  ROS_INFO("TTC= %f", g_ttc); // For debugging: print TTC value

  ROS_INFO("Relative Velocity= %f", g_relative_vel); // For debugging: print relative velocity between target vehicle and ego vehicle
  

  ROS_INFO("Steering angle = %f radians", twist_cmd_placeholder);
  twist_cmd.angular.z = twist_cmd_placeholder;

  pub.publish(twist_cmd);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "audibot_acc");

  if (stage == 1) { 
  double displacement = std::numeric_limits<double>::infinity(); // initialize to infinity
  }

  // Ceate a nodehandle for the acc node
  ros::NodeHandle node_handle;
  
  // Subscriber to "gazebo/model_states"
  ros::Subscriber model_states_subscriber = node_handle.subscribe("/gazebo/model_states", 1, modelStatesCallbackFunction);
  if (stage == 0) {
    ROS_INFO("Subscribed to models states (Stage 1)...");
  }
  
  // Subscriber to "/ego_vehicle/laser/scan"
  ros::Subscriber lidar_subscriber = node_handle.subscribe("/ego_vehicle/laser/scan", 1, lidarCallbackFunction);  
  if (stage == 1) {
   ROS_INFO("Subscribed to laser scan (Stage 2)...");
  }

  // Set up publishers and subscribers
  pub = node_handle.advertise<geometry_msgs::TwistStamped>("/cmd_vel", 1);
  ros::Subscriber sub_laser = node_handle.subscribe("/scan", 1, lidarCallbackFunction); // Subscribe to LIDAR sensor data
  stage = 1; // Set the stage variable to 1 for Level 2

  // Subscriber to "/ego_vehicle/front_camera/image_rect_color"
  ros::Subscriber camera_subscriber = node_handle.subscribe("/ego_vehicle/front_camera/image_raw", 1, cameraCallbackFunction);  
  if (stage == 2) {
   ROS_INFO("Subscribed to camera (Stage 3)...");
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