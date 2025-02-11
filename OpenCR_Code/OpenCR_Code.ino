#include "ros_functions.h"
#include "motor_functions.h"

DynamixelWorkbench dxl_wb;
ros::NodeHandle nh;
int pos;
void setup() {
  Serial.begin(921600);
  ros_inti();
  intiDyn();
  initSort();
  initMotorQueue();
  setupWheels();

  
  dxl_wb.torqueOff(DXL_SCOOP_L);
  dxl_wb.torqueOff(DXL_SCOOP_R);
  dxl_wb.itemWrite(DXL_SCOOP_L, "Profile_Acceleration", 5);
  dxl_wb.itemWrite(DXL_SCOOP_R, "Profile_Acceleration", 5);
  dxl_wb.torqueOn(DXL_SCOOP_L);
  dxl_wb.torqueOn(DXL_SCOOP_R);
  
  homeAll();
  //scoopUp();

  
  //autoHome(DXL_TILT);
  // initMotor(DXL_DUMP_R,EXT_POSITION_CONTROL_MODE);
  // dxl_wb.itemWrite(DXL_DUMP_R, "Velocity_Limit", 50);
  // setPos(DXL_DUMP_R,2000);
  // initMotor(DXL_DUMP_L,EXT_POSITION_CONTROL_MODE);
  // dxl_wb.itemWrite(DXL_DUMP_L, "Velocity_Limit", 50);
  // setPos(DXL_DUMP_L,600);
  //autoHome(DXL_DUMP_R);

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
  updateMotors();
  stateMachine();
  //getPos(DXL_SCOOP_R,pos);
  //setPos(DXL_SCOOP_R,SCOOP_R_UP_POS);
  //getPos(DXL_SCOOP_R,pos);
  publishMotor1Data();
  publishMotor2Data();
  publishMotor3Data();
  publishMotor4Data();
  readAndPublishIMU();
  delay(SAMPLING_PERIOD);
}
