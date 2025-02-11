#include <ros.h>
#include "ros_functions.h"
#include "motor_functions.h"
#include <sensor_msgs/Imu.h>
#include <std_msgs/UInt16.h>
#include <IMU.h>

DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;
cIMU imu;
sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

// Declare the variables for the orientation correction
std_msgs::Float32 z_deg_prev;
std_msgs::Float32 z_degrees;

void setup() {
  Serial.begin(57600);
  ros_init();
  setupDynWheels();
  autoHome(5);
  sortInit();

  nh.initNode();
  nh.advertise(imu_pub);
  
  imu.begin();

  // Initialization of the z_deg_prev and z_degrees variables
  z_deg_prev.data = imu.quat[3] * 180.0f; // Assuming quat[3] represents the z-axis rotation
  z_degrees.data = z_deg_prev.data;
}

void loop() {
  nh.spinOnce();
  imu.update();
  publishMotor1Data();
  publishMotor2Data();
  publishMotor3Data();
  publishMotor4Data();

  if (millis() - pre_time >= SAMPLING_PERIOD) {
    pre_time = millis(); // Only update pre_time once

    imu_msg.header.stamp    = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imu.gyroData[0];
    imu_msg.angular_velocity.y = imu.gyroData[1];
    imu_msg.angular_velocity.z = imu.gyroData[2];
    imu_msg.angular_velocity_covariance[0] = 0.02;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 0.02;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 0.02;

    imu_msg.linear_acceleration.x = imu.accData[0];
    imu_msg.linear_acceleration.y = imu.accData[1];
    imu_msg.linear_acceleration.z = imu.accData[2];
    imu_msg.linear_acceleration_covariance[0] = 0.04;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 0.04;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 0.04;

    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];

    // Correct z-axis rotation based on gyro data
    if (imu.gyroData[2] > 0) {
      if (imu.quat[3] < z_deg_prev.data) {
        z_degrees.data = imu.quat[3] * 180.0f;
      }
      else if (imu.quat[3] > z_deg_prev.data) {
        z_degrees.data = imu.quat[3] * -180.0f;
      }
    }
    if (imu.gyroData[2] < 0) {
      if (imu.quat[3] > z_deg_prev.data) {
        z_degrees.data = imu.quat[3] * 180.0f;
      }
      else if (imu.quat[3] < z_deg_prev.data) {
        z_degrees.data = imu.quat[3] * -180.0f;
      }
    }

    // Update previous z-axis degree for next loop iteration
    z_deg_prev.data = z_degrees.data;

    imu_pub.publish(&imu_msg); // Publish IMU data
  }

  delay(SAMPLING_PERIOD); // Define your sampling period here
}
