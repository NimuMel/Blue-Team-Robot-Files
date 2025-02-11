#include "ros_functions.h"
#include "macros.h"



char sort_control;
bool sort_done;
cIMU imu;
std_msgs::Float32 motor1_data, motor2_data, motor3_data, motor4_data;
std_msgs::Bool sort_done_msg;
sensor_msgs::Imu imu_msg;
std_msgs::Float32 yaw_msg;
std_msgs::Char sort_status;

ros::Publisher motor1_pub("/enc/motor1_encoder", &motor1_data);
ros::Publisher motor2_pub("/enc/motor2_encoder", &motor2_data);
ros::Publisher motor3_pub("/enc/motor3_encoder", &motor3_data);
ros::Publisher motor4_pub("/enc/motor4_encoder", &motor4_data);
ros::Publisher imu_pub("/imu/data_raw", &imu_msg);
ros::Publisher yaw_pub("/imu/yaw", &yaw_msg);
ros::Publisher scoop_done_pub("/OpenCR/scoop_done", &sort_done_msg);
ros::Publisher Dyn_cntl_pub("/servo/scoop_pose/Status", &sort_status);

//ros::Subscriber<std_msgs::Char>  Dyn_cntl_sub("/OpenCR/servo/control_sub", control_callback);
ros::Subscriber<std_msgs::Float64> w1_topic("/OpenCR/wheel_speeds/wheel_1_speed", wheel_1_write);
ros::Subscriber<std_msgs::Float64> w2_topic("/OpenCR/wheel_speeds/wheel_2_speed", wheel_2_write);
ros::Subscriber<std_msgs::Float64> w3_topic("/OpenCR/wheel_speeds/wheel_3_speed", wheel_3_write);
ros::Subscriber<std_msgs::Float64> w4_topic("/OpenCR/wheel_speeds/wheel_4_speed", wheel_4_write);
ros::Subscriber<std_msgs::Bool> scoop_request_sub("/OpenCR/scoop", scoop_requested);

uint8_t dxl_FL = DXL_FL_ID;
uint8_t dxl_FR = DXL_FR_ID;
uint8_t dxl_RL = DXL_RL_ID;
uint8_t dxl_RR = DXL_RR_ID;
bool scoop_request = false;

void control_callback(const std_msgs::Char& msg){
  sort_control = msg.data;
}

void pubScoopDone(bool status){
	sort_done_msg.data = status;
	scoop_done_pub.publish(&sort_done_msg);
}

void ros_inti() {
  nh.initNode();

  nh.subscribe(w1_topic);
  nh.subscribe(w2_topic);
  nh.subscribe(w3_topic);
  nh.subscribe(w4_topic);
  nh.subscribe(scoop_request_sub);
  nh.advertise(motor1_pub);
  nh.advertise(motor2_pub);
  nh.advertise(motor3_pub);
  nh.advertise(motor4_pub);
  nh.advertise(scoop_done_pub);
  nh.advertise(imu_pub);
  nh.advertise(yaw_pub);

  imu.begin();
  
}

int32_t radToDyn(const std_msgs::Float64& msg){
  if(msg.data >= 1023) return 1023;
  if(msg.data <= -1023) return -1023;
  return msg.data;
}
void scoop_requested(const std_msgs::Bool& msg){
	scoop_request = msg.data;
}
void wheel_1_write(const std_msgs::Float64& msg) {

  dxl_wb.goalVelocity(2, radToDyn(msg));
}
void wheel_2_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(3, radToDyn(msg));
}
void wheel_3_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(4, radToDyn(msg));
}
void wheel_4_write(const std_msgs::Float64& msg) {
  dxl_wb.goalVelocity(1, radToDyn(msg));
}

// The publishers are remapped for the topic becasue of math
//id 1 =wheel 2 
//id 2 = wheel 3 
//id 3 = wheel 4
//id 4 = wheel 1
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
  motor2_pub.publish(&motor3_data);
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
  motor3_pub.publish(&motor4_data);
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
  motor4_pub.publish(&motor1_data);
}


void readAndPublishIMU() {
  static uint32_t tTime[3];
  static uint32_t imu_time = 0;

 // tTime[2] = micros();
  //if (IMU.update() > 0) imu_time = micros() - tTime[2];

  if ((millis() - tTime[1]) >= 20) {
    imu.update();
    tTime[1] = millis();

    imu_msg.header.stamp    = nh.now();
    imu_msg.header.frame_id = "imu_link";

    imu_msg.angular_velocity.x = imu.gyroData[0];
    imu_msg.angular_velocity.y = imu.gyroData[1];
    imu_msg.angular_velocity.z = imu.gyroData[2];
    imu_msg.angular_velocity_covariance[0] = 999;
    imu_msg.angular_velocity_covariance[1] = 0;
    imu_msg.angular_velocity_covariance[2] = 0;
    imu_msg.angular_velocity_covariance[3] = 0;
    imu_msg.angular_velocity_covariance[4] = 999;
    imu_msg.angular_velocity_covariance[5] = 0;
    imu_msg.angular_velocity_covariance[6] = 0;
    imu_msg.angular_velocity_covariance[7] = 0;
    imu_msg.angular_velocity_covariance[8] = 999;

    imu_msg.linear_acceleration.x = imu.accData[0];
    imu_msg.linear_acceleration.y = imu.accData[1];
    imu_msg.linear_acceleration.z = imu.accData[2];
    imu_msg.linear_acceleration_covariance[0] = 999;
    imu_msg.linear_acceleration_covariance[1] = 0;
    imu_msg.linear_acceleration_covariance[2] = 0;
    imu_msg.linear_acceleration_covariance[3] = 0;
    imu_msg.linear_acceleration_covariance[4] = 999;
    imu_msg.linear_acceleration_covariance[5] = 0;
    imu_msg.linear_acceleration_covariance[6] = 0;
    imu_msg.linear_acceleration_covariance[7] = 0;
    imu_msg.linear_acceleration_covariance[8] = 999;

    imu_msg.orientation.w = imu.quat[0];
    imu_msg.orientation.x = imu.quat[1];
    imu_msg.orientation.y = imu.quat[2];
    imu_msg.orientation.z = imu.quat[3];

    imu_msg.orientation_covariance[0] = 999;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 999;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.1;

    // Publish the IMU message
    imu_pub.publish(&imu_msg);

    // Publish the yaw angle
    yaw_msg.data = imu.rpy[2]; // Assuming IMU.rpy[2] is yaw in radians
    yaw_pub.publish(&yaw_msg);

    //Optional: Print debug information to Serial Monitor
    Serial.print("IMU Time: ");
    Serial.print(imu_time);
    Serial.print(" us\t");

    Serial.print("Acc: [");
    Serial.print(imu.accData[0]);
    Serial.print(", ");
    Serial.print(imu.accData[1]);
    Serial.print(", ");
    Serial.print(imu.accData[2]);
    Serial.print("] m/s^2\t");

    Serial.print("Gyro: [");
    Serial.print(imu.gyroData[0]);
    Serial.print(", ");
    Serial.print(imu.gyroData[1]);
    Serial.print(", ");
    Serial.print(imu.gyroData[2]);
    Serial.print("] rad/s\t");


    Serial.print("Yaw: ");
    Serial.print(imu.rpy[2]);
    Serial.println(" rad");
  }
}