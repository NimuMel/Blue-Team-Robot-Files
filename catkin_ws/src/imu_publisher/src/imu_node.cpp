#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <IMU.h>

// Node handle
ros::NodeHandle nh;

// IMU message
sensor_msgs::Imu imu_msg;

// Publisher for IMU data
ros::Publisher imu_pub("imu", &imu_msg);

// IMU object
cIMU imu;

// Sampling period in milliseconds
#define SAMPLING_PERIOD 50

void setup()
{
  // Initialize serial communication
  Serial.begin(57600);

  // Initialize ROS node
  nh.initNode();
  nh.advertise(imu_pub);

  // Initialize IMU
  imu.begin();
}

void loop()
{
  static uint32_t pre_time = 0;

  // Update IMU data
  imu.update();

  if (millis() - pre_time >= SAMPLING_PERIOD)
  {
    pre_time = millis();

    // Populate IMU message
    imu_msg.header.stamp    = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imu.gyroData[0];
    imu_msg.angular_velocity.y = imu.gyroData[1];
    imu_msg.angular_velocity.z = imu.gyroData[2];
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = imu.accData[0];
    imu_msg.linear_acceleration.y = imu.accData[1];
    imu_msg.linear_acceleration.z = imu.accData[2];
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];
    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[8] = 0.0025;

    // Publish IMU data
    imu_pub.publish(&imu_msg);

    nh.spinOnce();
  }
}
