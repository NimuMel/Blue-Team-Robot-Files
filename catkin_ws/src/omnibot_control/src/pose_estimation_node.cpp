#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>
#include <unistd.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

#define ALPHA 0.98
#define BETA 0.99999
#define IMU_AVG 15
// Robot-specific constants
const float WHEEL_RADIUS = 0.054 / 2;  // Wheel radius in meters
const float ROBOT_RADIUS = 0.24 / 2;   // Robot radius in meters
const float TWO_PI = 2.0 * M_PI;
#define HZ 30
const float DT = 1.0/HZ;  // Time step (assumes 50 Hz update rate)

// Global pose
geometry_msgs::Pose2D robot_pose;
geometry_msgs::Quaternion odom_quat;

//encoder values 
std::vector<double> previous_encoder_values(4, 0.0);
std::vector<double> current_encoder_values(4, 0.0);

bool is_initialized = false;

float imu_yaw = 0;
float imu_yaw_speed = 0;
float imu_offset = 0;
float prev_imu_yaw_filtered = 0.0;

float prev_imu_yaw = 0.0; // Previous IMU reading
float imu_yaw_filtered = 0.0; // Filtered IMU yaw
float imu_yaw_bandpassed = 0.0;

float imu_avg = 0;
int imu_num_dat = 0;
float imu_yaw_avg[IMU_AVG];
ros::Time current_time;

// Callback functions for each encoder
void encoder1Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[0] = msg->data;
}

void encoder2Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[1] = msg->data;
}

void encoder3Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[2] = msg->data;
}

void encoder4Callback(const std_msgs::Float32::ConstPtr& msg) {
  current_encoder_values[3] = msg->data;
}
void imuCallback(const std_msgs::Float32::ConstPtr& msg) {
	 float imu_raw = (float)msg->data * 0.01745; // Convert to radians

    // Apply High-Pass Filter (removes drift)
    imu_yaw_filtered = .999 * (imu_yaw_filtered + imu_raw - prev_imu_yaw);
     prev_imu_yaw = imu_raw;
    // Apply Low-Pass Filter (removes high-frequency noise)
	imu_yaw_bandpassed = BETA * imu_yaw_bandpassed + (1 - BETA) * imu_yaw_filtered;
  
    imu_yaw = imu_raw; // Final band-passed IMU yaw
	ROS_INFO("IMU Raw: %.4f rad, IMU Filtered: %.4f rad", imu_raw, imu_yaw);
}



void calculatePose(ros::Publisher& pose_pub, ros::Publisher& odom_pub) {
  current_time = ros::Time::now();
  if (!is_initialized) {
    previous_encoder_values = current_encoder_values;
	prev_imu_yaw_filtered = imu_yaw_bandpassed;
	imu_offset = imu_yaw;
    is_initialized = true;
    ROS_INFO("Pose estimation initialized with encoder values.");
  }

  // Calculate wheel angular velocities from encoder values
  std::vector<double> wheel_speeds(4, 0.0);
  for (size_t i = 0; i < 4; ++i) {
    wheel_speeds[i] = (current_encoder_values[i] - previous_encoder_values[i]) / DT;
  }
	
  // Update previous encoder values
	previous_encoder_values = current_encoder_values;
	
	
	imu_yaw_speed = (imu_yaw_bandpassed - prev_imu_yaw_filtered);
    prev_imu_yaw_filtered = imu_yaw_bandpassed; // Store for next iteration


  // Compute the robot's linear velocity (Vx, Vy) and angular velocity (omega)
  double Vx = 0, Vy = 0, omega = 0, yaw = 0;
	Vx = ((WHEEL_RADIUS * sqrt(2)) / 4.0) * ( -wheel_speeds[0] - wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3] );
    Vy = ((WHEEL_RADIUS * sqrt(2)) / 4.0) * ( wheel_speeds[0] - wheel_speeds[1] - wheel_speeds[2] + wheel_speeds[3] );
    omega = -1*(WHEEL_RADIUS / (4.0 *ROBOT_RADIUS )) * (wheel_speeds[0] + wheel_speeds[1] + wheel_speeds[2] + wheel_speeds[3] );
	
	
  // Update robot pose
  robot_pose.x += Vx * DT * cos(robot_pose.theta) - Vy * DT * sin(robot_pose.theta);
  robot_pose.y += Vx * DT * sin(robot_pose.theta) + Vy * DT * cos(robot_pose.theta);
  robot_pose.theta += ALPHA * (omega * DT) + (1 - ALPHA) * (imu_yaw - imu_offset);

  // Normalize theta to [-pi, pi]
  if (robot_pose.theta > M_PI)
    robot_pose.theta -= TWO_PI;
  if (robot_pose.theta < -M_PI)
    robot_pose.theta += TWO_PI;

  odom_quat = tf::createQuaternionMsgFromYaw(robot_pose.theta);

  // Broadcast the transformation
  


  // Publish the odometry message
  nav_msgs::Odometry odom;
  odom.header.stamp = current_time;
  odom.header.frame_id = "start";
  odom.child_frame_id = "odom";
  // Set position
  odom.pose.pose.position.x = robot_pose.x;
  odom.pose.pose.position.y = robot_pose.y;
  odom.pose.pose.position.z = 0.0;
  odom.pose.pose.orientation = odom_quat;
	odom.pose.covariance = { 
		0.001,  0,    0,    0,    0,    0,  
		0,    0.001,  0,    0,    0,    0,  
		0,    0,    999,  0,    0,    0,  
		0,    0,    0,    999,  0,    0,  
		0,    0,    0,    0,    999,  0,  
		0,    0,    0,    0,    0,    0.001  
	};
  // Set velocity
  
  odom.twist.twist.linear.x = Vx;
  odom.twist.twist.linear.y = Vy;
  odom.twist.twist.angular.z = omega;
	odom.twist.covariance = { 
    999,  0,    0,    0,    0,    0,  
    0,    999,  0,    0,    0,    0,  
    0,    0,    999,  0,    0,    0,  
    0,    0,    0,    999,  0,    0,  
    0,    0,    0,    0,    999,  0,  
    0,    0,    0,    0,    0,    999  
};
  // Broadcast the transformation
  



  // Publish the updated pose
  
  pose_pub.publish(robot_pose);
  odom_pub.publish(odom);

}

void updateTf(tf::TransformBroadcaster& odom_broadcaster){

  //odom transformation
  geometry_msgs::TransformStamped odom_trans;
  odom_trans.header.stamp = current_time;
  odom_trans.header.frame_id = "start";
  odom_trans.child_frame_id = "odom";

  odom_trans.transform.translation.x = robot_pose.x;
  odom_trans.transform.translation.y = robot_pose.y;
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = odom_quat;

  odom_broadcaster.sendTransform(odom_trans);
}

int main(int argc, char** argv) {
  // Initialize the ROS node
 ros::init(argc, argv, "pose_estimation_node");
    ros::NodeHandle nh;

    // Instantiate ROS publishers and subscribers after ros::init()
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("robot_pose", 10);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);

    ros::Subscriber encoder1_sub = nh.subscribe("/enc/motor1_encoder", 10, encoder1Callback);
    ros::Subscriber encoder2_sub = nh.subscribe("/enc/motor2_encoder", 10, encoder2Callback);
    ros::Subscriber encoder3_sub = nh.subscribe("/enc/motor3_encoder", 10, encoder3Callback);
    ros::Subscriber encoder4_sub = nh.subscribe("/enc/motor4_encoder", 10, encoder4Callback);
	
	//add sub for imu yaw
	ros::Subscriber imu_sub_ = nh.subscribe("/imu/yaw", 10, imuCallback);
  // Instantiate Transform Broadcasters
    tf::TransformBroadcaster odom_broadcaster;
    tf::TransformBroadcaster pose_broadcaster;
  // Initialize robot pose
  robot_pose.x = 0.0;
  robot_pose.y = 0.0;
  robot_pose.theta = 0.0;

  // Set the update rate
  ros::Rate rate(HZ);

  while (ros::ok()) {
   if (!is_initialized) {
      sleep(5);
    }
    ros::spinOnce();
    calculatePose(pose_pub, odom_pub);
	updateTf(odom_broadcaster);
    rate.sleep();
  }

  return 0;
}
