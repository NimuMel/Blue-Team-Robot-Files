#include "ros_functions.h"
#include "motor_functions.h"

DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;

void setup() {
  Serial.begin(57600);
  ros_inti();
  setupDynWheels();
  // autoHome(5);
  // autoHomeScoop();
  // autoHomeDump();
  // sortInit();
  // delay(2000);
  // unsigned long startTime = micros(); // Record the start time
  // scoopUp();
  // delay(1000);
  
  // scoopDown();
  // tiltUp();
  // sortRight();
  // sortRight();
  // sortRight();
  // sortRight();
  // sortRight();
  // sortRight();
  // sortRight();
  // sortRight();
  // // sortLeft();
  // // sortRight();
  // // sortLeft();
  // // sortRight();
  // tiltDown();
  // dumpUp();
  // delay(4000);
  // dumpDown();
  // unsigned long endTime = micros(); // Record the end time

  // unsigned long elapsedTime = endTime - startTime; // Calculate elapsed time
  // Serial.print("Elapsed time: ");
  // Serial.print(elapsedTime/100000);
  // Serial.println(" deciseconds");
}

void loop() {
  nh.spinOnce();
  publishMotor1Data();
  publishMotor2Data();
  publishMotor3Data();
  publishMotor4Data();
  delay(SAMPLING_PERIOD);
}
