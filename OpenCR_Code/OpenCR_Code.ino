#include "ros_functions.h"
#include "motor_functions.h"

DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;

void setup() {
  Serial.begin(57600);
  ros_inti();
  setupDynWheels();
  autoHome(5);
  sortInit();
  // tiltUp();
  // sortRight();
  // sortLeft();
  // sortRight();
  // sortLeft();
  // sortRight();
  // sortLeft();
  // sortRight();
  // tiltDown();
  //dxl_wb.goalVelocity(1, 10);
}

void loop() {
  nh.spinOnce();
  publishMotor1Data();
  publishMotor2Data();
  publishMotor3Data();
  publishMotor4Data();
  delay(SAMPLING_PERIOD);
}
