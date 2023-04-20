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
#include <iostream> 
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdexcept>
#include <string>
#include <math.h>

// jseablom (jseablom)
// Kacper Wojtowicz (KacoerWijtowicz)
// Program to implement adaptive cruise control ACC feature
// Version 1.1 
// Date: 4/2/2023
// Cleaned up code/comments and fixed errors in previous version
// Authors (github names):
// jseablom (N/A)
// Kacper Wojtowicz (KacperWojtowicz)
// YaswanthBoppana (rogueassassin14)
// Kristof von Czarnowski (kristofvonc)

int stage = 0;
int speed_algo = 1;
//set 'stage' variable to:
// 0 - Level 1: Implement core system with ideal measurements
// 1 - Level 2: Use a LIDAR sensor to detect the lead vehicle
// 2 - Level 3: Use the camera to detect the lead vehicle

double refresh_rate = 0.1;

// Set the desired following distance for adaptive cruise control
const double following_distance = 10.0; 
if (speed_algo == 1) {
  following_distance = 5.5;
}
double following_error_margin_multiplier = 0.0;

int seg_length;
int focal;

//Create geoemtry messages 
geometry_msgs::Pose target_vehicle_position;
geometry_msgs::Pose ego_vehicle_position; 
geometry_msgs::Twist target_vehicle_vel;
geometry_msgs::Twist ego_vehicle_vel; 
geometry_msgs::Twist twist_cmd;
//Create static publishers
static ros::Publisher pub; 
static ros::Subscriber sub_laser;

double twist_cmd_placeholder = 0;
double displacement = 100;
double relative_vel = 0;
double ttc = 0; 
//image_geometry::PinholeCameraModel PinholeCameraModel;
double camera_height = 2.0; // Height of the ego camera from the ground plane in meters
double camera_pitch = 0.275643; // Pitch angle of the ego camera in radians
double camera_fov = 1.3962634; // Field of view of the ego camera in radians
double lead_vehicle_height = 1.0; // Height of the lead vehicle from the ground plane in meters

// The value of '1' may need to be 'tuned' based on vehicle dynamics (braking/acceleration ability of audibot)


// Callback function whenever a new /gazebo/model_states message is received
void modelStatesCallbackFunction(const gazebo_msgs::ModelStates msg) {
  ego_vehicle_position  = msg.pose[5];
  target_vehicle_position = msg.pose[6];

  ego_vehicle_vel  = msg.twist[5];
  target_vehicle_vel = msg.twist[6];
}

// Callback function whenever a new lidar scan message is received
void lidarCallbackFunction(const sensor_msgs::LaserScan::ConstPtr& msg) {
  if (stage == 1) {
  displacement = *std::min_element(msg->ranges.begin(), msg->ranges.end());
  }
}

//void cameraCallbackFunction2(const sensor_msgs::CameraInfoConstPtr& msg2) {
//  PinholeCameraModel.fromCameraInfo(msg2);
//  focal=PinholeCameraModel.fy();
//  ROS_INFO_STREAM("fy: " << focal);
//}

// Callback function whenever a new camera image is received
void cameraCallbackFunction(const sensor_msgs::Image::ConstPtr& msg) {
  if (stage == 2) {
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
  //cv::imshow("Blue Image", blue_image);
  cv::waitKey(1);
  // Apply binary threshold to create a binary image where white pixels correspond to high blue values
  cv::Mat thres_img;
  cv::threshold(blue_image, thres_img, 20, 255, cv::THRESH_BINARY);
  //cv::Mat thres_img;
  cv::Mat t1;
  cv::Mat t2;
  cv::Mat t3;
  cv::Mat t4;
  cv::threshold(red_image, t1, 50, 256, cv::THRESH_BINARY);
  cv::threshold(red_image, t2, 80, 256, cv::THRESH_BINARY_INV);
  cv::bitwise_and(t1, t2, thres_img);
  //cv::imshow("Thres Image", thres_img);
  cv::waitKey(1);
  // Apply erosion to clean up noise
  cv::Mat erode_img;
  cv::erode(thres_img, erode_img, cv::Mat::ones(5, 5, CV_8U));
  //cv::imshow("Erode Image", erode_img);
  cv::waitKey(1);
  // Apply dilation to expand regions that passed the erosion filter
  cv::Mat dilate_img;
  cv::dilate(erode_img, dilate_img, cv::Mat::ones(5, 5, CV_8U));
  //cv::imshow("Dilate Image", dilate_img);
  cv::waitKey(1);
  // Apply Canny edge detection to reduce the number of points that are passed to Hough Transform
  cv::Mat canny_img;
  cv::Canny(dilate_img, canny_img, 1, 2);
  //cv::imshow("Canny Image", canny_img);
  std::vector<cv::Vec4i> line_segments;
  cv::HoughLinesP(canny_img, line_segments,  10, 0.05, 2, 10, 50);  
  // Draw detected Hough lines onto the raw image for visualization
  for (int i=0; i<line_segments.size(); i++){
    cv::line(raw_img, cv::Point(line_segments[i][0], line_segments[i][1]),
      cv::Point(line_segments[i][2], line_segments[i][3]), cv::Scalar(0, 255, 0));
      float seg_y = line_segments[i][0] - line_segments[i][2];
      float seg_x = line_segments[i][1] - line_segments[i][3];

      seg_length = sqrt(pow(seg_y,2) + pow(seg_x,2));
      //ROS_INFO_STREAM("Line Length in Pixels: " << seg_length);
      //ROS_INFO_STREAM("Displacement: " << displacement);
      //linear_speed = 5; 
      //pub.publish(twist_cmd);
      
  }
  cv::imshow("Lines Image", raw_img);
  cv::waitKey(1);
  int real_height=800;
  int obj_height = 2;
  int sensor_height = 2;
  int image_height = 50; 
  }

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

    double vx = target_vehicle_vel.linear.x - ego_vehicle_vel.linear.x;
    relative_vel = abs(vx);
  }

  if (stage == 1) {
    if (displacement == std::numeric_limits<double>::infinity()) {
      ROS_INFO("Waiting for LIDAR scan message to be received...");
    return;
    }

    double vx = target_vehicle_vel.linear.x - ego_vehicle_vel.linear.x;
    relative_vel = abs(vx);
  }

  if (stage == 2) {
    //PLACEHOLDER
    double dx = target_vehicle_position.position.x - ego_vehicle_position.position.x;
    double dy = target_vehicle_position.position.y - ego_vehicle_position.position.y;
    double dz = target_vehicle_position.position.z - ego_vehicle_position.position.z;
    displacement = std::sqrt(dx*dx + dy*dy + dz*dz);

    double vx = target_vehicle_vel.linear.x - ego_vehicle_vel.linear.x;
    relative_vel = abs(vx);
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
  if (speed_algo == 0) {
  if (displacement > (1+following_error_margin_multiplier)*following_distance) {
   linear_speed = displacement / ((1+following_error_margin_multiplier)*following_distance) * linear_speed; // Speed up - Open road
  } else if (displacement > following_distance && displacement <= (1+following_error_margin_multiplier)*following_distance) {
   linear_speed = 13.1; // Traffic nearby - Maintain (default) speed 
  } else if (displacement > following_distance/(1+following_error_margin_multiplier) && displacement <= following_distance) {
   linear_speed = displacement / following_distance * linear_speed; // Getttingc closer- Slow down slightly
  } else {
   linear_speed = 0; // Too close - Slow down quickly
  }
  }

  if (speed_algo == 1) {
// Calculate the linear speed based on TTC and desired following distance
     double desired_ttc = 2.5;  // Desired time-to-collision in seconds
     ttc = displacement / relative_vel;
     if (ttc > desired_ttc) {
        linear_speed = (displacement / following_distance)*linear_speed;
      } else {
        linear_speed = 0;  // Stop the ego vehicle if TTC is less than desired TTC
      }
      //ego_vehicle_vel.linear.x = linear_speed;
  }
  ROS_INFO("Set linear speed = %f", linear_speed); // For debugging: print linear speed value
  twist_cmd.linear.x = linear_speed; 
  ROS_INFO("Steering angle = %f radians", twist_cmd_placeholder);
  twist_cmd.angular.z = twist_cmd_placeholder;
  pub.publish(twist_cmd);
  ROS_INFO("TTC= %f", ttc); // For debugging: print TTC value
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


 if (stage == 2) {
  //Names Windows
  cv::namedWindow("Raw Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Blue Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Green Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Red Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Thres Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Erode Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Dilate Image", cv::WINDOW_AUTOSIZE);
  //cv::namedWindow("Canny Image", cv::WINDOW_AUTOSIZE);
  cv::namedWindow("Lines Image", cv::WINDOW_AUTOSIZE);
 }

  // Subscriber to "/ego_vehicle/front_camera/image_rect_color"
  ros::Subscriber camera_subscriber = node_handle.subscribe("/ego_vehicle/front_camera/image_raw", 1, cameraCallbackFunction);  
  if (stage == 2) {
   ROS_INFO("Subscribed to camera (Stage 3)...");
  }
  
  ros::Subscriber twist_subscriber = node_handle.subscribe("/ego_vehicle/twist", 1, twistTime);

  // Creat a timer for our node, currently set to 10 Hz (Argument specified in seconds)
  ros::Timer timer = node_handle.createTimer(ros::Duration(refresh_rate), timerCallback);

  // Creates a cmd_vel publisher
  //pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel2", 1);
  pub = node_handle.advertise<geometry_msgs::Twist>("/ego_vehicle/cmd_vel", 1);

  // Keep the node running
  ros::spin();

  //return 0;
}