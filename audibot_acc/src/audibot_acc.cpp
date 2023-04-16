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
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// jseablom (jseablom)
// Kacper Wojtowicz (KacoerWijtowicz)
// Program to implement adaptive cruise control ACC feature
// Version 1.1 
// Date: 4/2/2023
// Cleaned up code/comments and fixed errors in previous version
// Authors (github names):
// jseablom (N/A)
// Kacper Wojtowicz (KacperWojtowicz)
// YawanthBoppana (rogueassassin14)
// Kristof von Czarnowski (kristofvonc)

int stage = 2;
//set 'stage' variable to:
// 0 - Level 1: Implement core system with ideal measurements
// 1 - Level 2: Use a LIDAR sensor to detect the lead vehicle
// 2 - Level 3: Use the camera to detect the lead vehicle

// Set the desired following distance for adaptive cruise control
const double following_distance = 10.0; 

//Create geoemtry messages 
geometry_msgs::Pose target_vehicle_position;
geometry_msgs::Pose ego_vehicle_position; 
geometry_msgs::Twist twist_cmd;
//Create static publishers
static ros::Publisher pub; 
static ros::Subscriber sub_laser;

double twist_cmd_placeholder = 0;
double displacement = 100;

//Placement of ego vehicle camera
//(5,-5,2) units in the X, Y, and Z directions, respectively
//(0.275643, 2.35619) represent the rotation around the X (roll) and Y (pitch) axes in radians
double camera_height = 2.0; // Height of the ego camera from the ground plane in meters
double camera_pitch = 0.275643; // Pitch angle of the ego camera in radians
double camera_fov = 1.3962634; // Field of view of the ego camera in radians
double lead_vehicle_height = 1.0; // Height of the lead vehicle from the ground plane in meters

// The value of '1' may need to be 'tuned' based on vehicle dynamics (braking/acceleration ability of audibot)


// Callback function whenever a new /gazebo/model_states message is received
void modelStatesCallbackFunction(const gazebo_msgs::ModelStates msg) {
  ego_vehicle_position  = msg.pose[5];
  target_vehicle_position = msg.pose[6];
}

// Callback function whenever a new lidar scan message is received
void lidarCallbackFunction(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (stage == 1) {
  displacement = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  }
}




// Callback function whenever a new camera image is received
void cameraCallbackFunction(const sensor_msgs::Image::ConstPtr& msg) {
  // Convert raw image from ROS image message into a cv::Mat
  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
  cv::Mat raw_img = cv_ptr->image;

  cv::imshow("Raw Image", raw_img);
  cv::waitKey(1);

  // Split RGB image into its three separate channels
  std::vector<cv::Mat> split_images;
  cv::split(raw_img, split_images);

  // Extract the blue channel into its own grayscale image
  cv::Mat blue_image = split_images[0];
  cv::Mat green_image = split_images[1];
  cv::Mat red_image = split_images[2];


  cv::imshow("Blue Image", blue_image);
  cv::waitKey(1);
  /*
  cv::imshow("Red Image", red_image);
  cv::waitKey(1);

    cv::imshow("Green Image", green_image);
  cv::waitKey(1);
  */
  // Apply binary threshold to create a binary image where white pixels correspond to high blue values
  cv::Mat thres_img;
  cv::threshold(blue_image, thres_img, 50, 100, cv::THRESH_BINARY);

  cv::imshow("Thres Image", thres_img);
  cv::waitKey(1);

  // Apply erosion to clean up noise
  cv::Mat erode_img;
  cv::erode(thres_img, erode_img, cv::Mat::ones(5, 5, CV_8U));

  cv::imshow("Erode Image", erode_img);
  cv::waitKey(1);

  // Apply dilation to expand regions that passed the erosion filter
  cv::Mat dilate_img;
  cv::dilate(erode_img, dilate_img, cv::Mat::ones(5, 5, CV_8U));

  cv::imshow("Dilate Image", dilate_img);
  cv::waitKey(1);

  // Apply Canny edge detection to reduce the number of points that are passed to Hough Transform
  cv::Mat canny_img;
  cv::Canny(dilate_img, canny_img, 1, 2);
  cv::imshow("Canny Image", canny_img);

    
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
  }

  if (stage == 1) {
    if (displacement == std::numeric_limits<double>::infinity()) {
      ROS_INFO("Waiting for LIDAR scan message to be received...");
    return;
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
    linear_speed = 13.1;
  }

  // Basic control algorithm, may want to implement PID controller, as Yaswanth was suggesting...
  // We may also want to control our speed based on time-to-collision (or 'TTC') or relative velocity.

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
  twist_cmd.linear.x = linear_speed; 

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


  //Names Windows
  cv::namedWindow("Raw Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Blue Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Green Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Red Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Thres Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Erode Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Dilate Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Canny Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Lines Image", cv::WINDOW_AUTOSIZE);

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