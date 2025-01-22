#include "ros_functions.h"
#include "macros.h"

extern DynamixelWorkbench dxl_wb;
extern ros::NodeHandle nh;

std_msgs::Float32 motor1_data, motor2_data, motor3_data, motor4_data;
ros::Publisher motor1_pub("motor1_encoder", &motor1_data);
ros::Publisher motor2_pub("motor2_encoder", &motor2_data);
ros::Publisher motor3_pub("motor3_encoder", &motor3_data);
ros::Publisher motor4_pub("motor4_encoder", &motor4_data);

ros::Subscriber<std_msgs::Float64> w1_topic("/wheel_1_speed", wheel_1_write);
ros::Subscriber<std_msgs::Float64> w2_topic("/wheel_2_speed", wheel_2_write);
ros::Subscriber<std_msgs::Float64> w3_topic("/wheel_3_speed", wheel_3_write);
ros::Subscriber<std_msgs::Float64> w4_topic("/wheel_4_speed", wheel_4_write);

uint8_t dxl_FL = DXL_FL_ID;
uint8_t dxl_FR = DXL_FR_ID;
uint8_t dxl_RL = DXL_RL_ID;
uint8_t dxl_RR = DXL_RR_ID;

void ros_inti() {
  nh.initNode();

  nh.subscribe(w1_topic);
  nh.subscribe(w2_topic);
  nh.subscribe(w3_topic);
  nh.subscribe(w4_topic);
  nh.advertise(motor1_pub);
  nh.advertise(motor2_pub);
  nh.advertise(motor3_pub);
  nh.advertise(motor4_pub);
}
void wheel_1_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(dxl_FR, (int32_t)msg.data);
}
void wheel_2_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(dxl_FL, (int32_t)msg.data);
}
void wheel_3_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(dxl_RR, (int32_t)msg.data);
}
void wheel_4_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(dxl_RL, (int32_t)msg.data);
}

void publishMotor1Data() {
  float radian;
  if (dxl_wb.getRadian(1, &radian)) {
    motor1_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 2");
    motor1_data.data = 0.0;
  }
  motor1_pub.publish(&motor2_data);
}

// Function to publish encoder data for Motor 2
void publishMotor2Data() {
  float radian;
  if (dxl_wb.getRadian(2, &radian)) {
    motor2_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 2");
    motor2_data.data = 0.0;
  }
  motor2_pub.publish(&motor2_data);
}

// Function to publish encoder data for Motor 3
void publishMotor3Data() {
  float radian;
  if (dxl_wb.getRadian(3, &radian)) {
    motor3_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 3");
    motor3_data.data = 0.0;
  }
  motor3_pub.publish(&motor3_data);
}

// Function to publish encoder data for Motor 4
void publishMotor4Data() {
  float radian;
  if (dxl_wb.getRadian(4, &radian)) {
    motor4_data.data = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 4");
    motor4_data.data = 0.0;
  }
  motor4_pub.publish(&motor4_data);
}