#include "motor_functions.h"
#include "ros_functions.h"
#include "macros.h"

int32_t tilt_home = 0;
int32_t sort_pos = 0;
int32_t tilt_pos = 0;

int32_t scoop_home_7 = 0;
int32_t scoop_home_8 = 0;

int32_t dump_1_home = 0;


extern ros::NodeHandle nh;
extern DynamixelWorkbench dxl_wb;  // Defined elsewhere in your program



void setupDynWheels() {

  //Local Variables for Error checking
  bool result = false;
  const char* log;

  // Initialize Dynamixel DEVICE_NAME
  result = dxl_wb.init("/dev/ttyACM0", BAUDRATE, &log);
  if (!result) {
    Serial.println("Failed to initialize Dynamixel");
    Serial.println(log);
    return;
  }
  Serial.println("Dynamixel Workbench initialized");

  // Set to Protocol 2.0
  dxl_wb.setPacketHandler(2.0);  // Use Protocol 2.0

  // Ping all motors and set them to velocity control mode
  uint16_t model_number = 0;
  uint8_t motor_ids[] = MOTOR_IDS;  // Motor IDs
  for (int i = 0; i < 4; i++) {
    delay(10);
    uint8_t motor_id = motor_ids[i];
    result = dxl_wb.ping(motor_id, &model_number, &log);
    if (!result) {
      Serial.print("Failed to ping motor ");
      Serial.print(motor_id);
      Serial.println(log);
      continue;  // Skip to the next motor
    }
    Serial.print("Motor ");
    Serial.print(motor_id);
    Serial.print(" pinged successfully. Model Number: ");
    Serial.println(model_number);
    delay(10);
    // Set the motor to Velocity Control Mode
    //ACCELERATION_MODE VELOCITY_CONTROL_MODE
    result = dxl_wb.setOperatingMode(motor_id, VELOCITY_CONTROL_MODE, &log);
    if (!result) {
      Serial.print("Failed to set velocity control mode for motor ");
      Serial.print(motor_id);
      Serial.println(log);
      continue;  // Skip to the next motor
    }
    Serial.print("Motor ");
    Serial.print(motor_id);
    Serial.println(" velocity control mode set successfully");
    delay(10);
    // Enable torque for the motor
    result = dxl_wb.torqueOn(motor_id, &log);
    if (!result) {
      Serial.print("Failed to enable torque for motor ");
      Serial.print(motor_id);
      Serial.println(log);
      continue;  // Skip to the next motor
    }
    Serial.print("Torque enabled for motor ");
    Serial.println(motor_id);
  }
}

void sortInit() {
  uint8_t motor_id = 6;
  bool result = false;
  const char* log;
  uint16_t model_number = 0;
  //Ping to see if connected
  result = dxl_wb.ping(motor_id, &model_number, &log);
  if (!result) {
    Serial.print("Failed to ping motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  Serial.print("Motor ");
  Serial.print(motor_id);
  Serial.print(" pinged successfully. Model Number: ");
  Serial.println(model_number);
  result = dxl_wb.setOperatingMode(motor_id, EXT_POSITION_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set velocity control mode for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  dxl_wb.itemWrite(motor_id, "Profile_Velocity", 0);
  dxl_wb.itemWrite(motor_id, "Profile_Acceleration", 0);
  result = dxl_wb.torqueOn(motor_id, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  Serial.print("Torque enabled for motor ");
  Serial.println(motor_id);
  result = dxl_wb.goalPosition(motor_id, 1022);  // Center
  if (!result) {
    Serial.print("Failed to set Position: ");
    Serial.print(motor_id);
    Serial.println(log);
  }
}

void autoHome(uint8_t motor_id) {
  //This function will home the all of the sort mechanisms
  bool result = false;
  const char* log;
  uint16_t model_number = 0;
  //Ping to see if connected
  result = dxl_wb.ping(motor_id, &model_number, &log);
  if (!result) {
    Serial.print("Failed to ping motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  //Set the mode to velocity control for homing
  result = dxl_wb.setOperatingMode(motor_id, VELOCITY_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set velocity control mode for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  result = dxl_wb.torqueOn(motor_id, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  result = dxl_wb.goalSpeed(motor_id, (int32_t)HOMING_VELOCITY, &log);
  if (!result) {
    Serial.print("Failed to set velocity: ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  Serial.println("Homing started");

  while (true) {
    int32_t present_load = 0;

    result = dxl_wb.itemRead(motor_id, "Present_Load", &present_load, &log);
    Serial.print("Present load:");
    Serial.println(present_load);
    if (!result) {
      Serial.print("Failed to get Present Load");
      Serial.print(motor_id);
      Serial.println(log);
    }
    // Check if the load exceeds the threshold
    if (abs(present_load) > LOAD_THRESHOLD) {
      Serial.println("Load threshold reached. Stopping homing.");
      break;
    }
  }
  // Stop the motor by setting the velocity to zero
  result = dxl_wb.goalSpeed(motor_id, 0, &log);
  if (!result) {
    Serial.print("Failed to stop motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
  }

  // Read the current position after homing

  result = dxl_wb.itemRead(motor_id, "Present_Position", &tilt_home, &log);
  if (!result) {
    Serial.print("Failed to read Present Position for motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  dxl_wb.torqueOff(motor_id);
  result = dxl_wb.setOperatingMode(motor_id, EXT_POSITION_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set Extended Position Control Mode for motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  Serial.print("Homing complete. Current position: ");
  Serial.println(tilt_home);

  result = dxl_wb.torqueOn(motor_id, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }
}

void autoHomeScoop() {
  int motor_7 = 7;
  int motor_8 = 8;
  bool result = false;
  const char* log;
  uint16_t model_number = 0;
  Serial.println("Scoop Homing");

  //Ping to see if connected
  result = dxl_wb.ping(motor_7, &model_number, &log);
  if (!result) {
    Serial.print("Failed to ping motor ");
    Serial.print(motor_7);
    Serial.println(log);
  }
  result = dxl_wb.ping(motor_8, &model_number, &log);
  if (!result) {
    Serial.print("Failed to ping motor ");
    Serial.print(motor_8);
    Serial.println(log);
  }


  //Set the mode to velocity control for homing
  result = dxl_wb.setOperatingMode(motor_7, VELOCITY_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set velocity control mode for motor ");
    Serial.print(motor_7);
    Serial.println(log);
  }
  result = dxl_wb.setOperatingMode(motor_8, VELOCITY_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set velocity control mode for motor ");
    Serial.print(motor_8);
    Serial.println(log);
  }


  //Set torque to on for Homing
  result = dxl_wb.torqueOn(motor_7, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_7);
    Serial.println(log);
  }
  result = dxl_wb.torqueOn(motor_8, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_8);
    Serial.println(log);
  }

  result = dxl_wb.goalSpeed(motor_7, -(int32_t)HOMING_VELOCITY, &log);
  if (!result) {
    Serial.print("Failed to set velocity: ");
    Serial.print(motor_7);
    Serial.println(log);
  }
  result = dxl_wb.goalSpeed(motor_8, (int32_t)HOMING_VELOCITY, &log);
  if (!result) {
    Serial.print("Failed to set velocity: ");
    Serial.print(motor_8);
    Serial.println(log);
  }
  // Homing While loop
  while (true) {
    int32_t present_load_7 = 0;
    int32_t present_load_8 = 0;

    //result = dxl_wb.itemRead(motor_7, "Present_Load", &present_load_7, &log);
    Serial.print("Present load 7:");
    Serial.println(present_load_7);
    if (!result) {
      Serial.print("Failed to get Present Load");
      Serial.print(motor_7);
      Serial.println(log);
    }
    result = dxl_wb.itemRead(motor_8, "Present_Load", &present_load_8, &log);
    Serial.print("Present load 8:");
    Serial.println(present_load_8);
    if (!result) {
      Serial.print("Failed to get Present Load");
      Serial.print(motor_8);
      Serial.println(log);
    }
    // Check if the load exceeds the threshold
    if (abs(present_load_7) > LOAD_THRESHOLD || abs(present_load_8) > LOAD_THRESHOLD) {
      Serial.println("Load threshold reached for scoop. Stopping homing.");
      break;
    }
  }


  // Stop the motor by setting the velocity to zero
  result = dxl_wb.goalSpeed(motor_7, 0, &log);
  if (!result) {
    Serial.print("Failed to stop motor ");
    Serial.print(motor_7);
    Serial.print(": ");
    Serial.println(log);
  }
  result = dxl_wb.goalSpeed(motor_8, 0, &log);
  if (!result) {
    Serial.print("Failed to stop motor ");
    Serial.print(motor_8);
    Serial.print(": ");
    Serial.println(log);
  }


  // Read the current position after homing
  result = dxl_wb.itemRead(motor_7, "Present_Position", &scoop_home_7, &log);
  if (!result) {
    Serial.print("Failed to read Present Position for motor ");
    Serial.print(motor_7);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  result = dxl_wb.itemRead(motor_8, "Present_Position", &scoop_home_8, &log);
  if (!result) {
    Serial.print("Failed to read Present Position for motor ");
    Serial.print(motor_8);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  Serial.print("Homing complete. Current position: ");
  Serial.println(scoop_home_7);
  Serial.print("Homing complete. Current position: ");
  Serial.println(scoop_home_8);

  dxl_wb.torqueOff(motor_7);
  dxl_wb.torqueOff(motor_8);
  result = dxl_wb.setOperatingMode(motor_7, EXT_POSITION_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set Extended Position Control Mode for motor ");
    Serial.print(motor_7);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  result = dxl_wb.setOperatingMode(motor_8, EXT_POSITION_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set Extended Position Control Mode for motor ");
    Serial.print(motor_8);
    Serial.print(": ");
    Serial.println(log);
    return;
  }


  result = dxl_wb.torqueOn(motor_7, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_7);
    Serial.println(log);
  }
  result = dxl_wb.torqueOn(motor_8, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_8);
    Serial.println(log);
  }
}

void autoHomeDump() {
  //This function will home the all of the sort mechanisms
  bool result = false;
  const char* log;
  uint16_t model_number = 0;
  int motor_id = 10;
  //Ping to see if connected
  result = dxl_wb.ping(motor_id, &model_number, &log);
  if (!result) {
    Serial.print("Failed to ping motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  //Set the mode to velocity control for homing
  result = dxl_wb.setOperatingMode(motor_id, VELOCITY_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set velocity control mode for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  result = dxl_wb.torqueOn(motor_id, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  result = dxl_wb.goalSpeed(motor_id, (int32_t)HOMING_VELOCITY, &log);
  if (!result) {
    Serial.print("Failed to set velocity: ");
    Serial.print(motor_id);
    Serial.println(log);
  }

  Serial.println("Homing started");

  while (true) {
    int32_t present_load = 0;

    result = dxl_wb.itemRead(motor_id, "Present_Current", &present_load, &log);
    Serial.print("Present load:");
    Serial.println(present_load);
    if (!result) {
      Serial.print("Failed to get Present Load");
      Serial.print(motor_id);
      Serial.println(log);
    }
    // Check if the load exceeds the threshold
    if (abs(present_load) > 300) {
      Serial.println("Load threshold reached. Stopping homing.");
      break;
    }
  }
  // Stop the motor by setting the velocity to zero
  result = dxl_wb.goalSpeed(motor_id, 0, &log);
  if (!result) {
    Serial.print("Failed to stop motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
  }

  // Read the current position after homing

  result = dxl_wb.itemRead(motor_id, "Present_Position", &dump_1_home, &log);
  if (!result) {
    Serial.print("Failed to read Present Position for motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  dxl_wb.torqueOff(motor_id);
  result = dxl_wb.setOperatingMode(motor_id, EXT_POSITION_CONTROL_MODE, &log);
  if (!result) {
    Serial.print("Failed to set Extended Position Control Mode for motor ");
    Serial.print(motor_id);
    Serial.print(": ");
    Serial.println(log);
    return;
  }
  Serial.print("Homing complete. Current position: ");
  Serial.println(dump_1_home);
  dxl_wb.itemWrite(motor_id, "Profile_Velocity", 50);
  result = dxl_wb.torqueOn(motor_id, &log);
  if (!result) {
    Serial.print("Failed to enable torque for motor ");
  }
}
void sortLeft() {
  uint8_t motor_id = 6;
  bool result = false;
  const char* log;
  dxl_wb.itemRead(motor_id, "Present_Position", &sort_pos, &log);
  result = dxl_wb.goalPosition(motor_id, -900);
  if (!result) {
    Serial.print("Failed to set Position: ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  while (abs(sort_pos + 900) > 5) {
    dxl_wb.itemRead(motor_id, "Present_Position", &sort_pos, &log);
  }
  result = dxl_wb.goalPosition(motor_id, 1022);  // Center
  if (!result) {
    Serial.print("Failed to set Position: ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  while (abs(sort_pos - 1022) > 5) {
    dxl_wb.itemRead(motor_id, "Present_Position", &sort_pos, &log);
  }
}

void sortRight() {
  uint8_t motor_id = 6;
  bool result = false;
  const char* log;
  dxl_wb.itemRead(motor_id, "Present_Position", &sort_pos, &log);
  result = dxl_wb.goalPosition(motor_id, 2900);
  if (!result) {
    Serial.print("Failed to set Position: ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  while (abs(sort_pos - 2900) > 5) {
    dxl_wb.itemRead(motor_id, "Present_Position", &sort_pos, &log);
  }
  result = dxl_wb.goalPosition(motor_id, 1022);  // Center
  if (!result) {
    Serial.print("Failed to set Position: ");
    Serial.print(motor_id);
    Serial.println(log);
  }
  while (abs(sort_pos - 1022) > 5) {
    dxl_wb.itemRead(motor_id, "Present_Position", &sort_pos, &log);
  }
}

void tiltUp() {
  bool result = false;
  const char* log;
  dxl_wb.goalPosition(5, tilt_home - 550);
  while (abs(tilt_pos - (tilt_home - 550)) > 30) {
    dxl_wb.itemRead(5, "Present_Position", &tilt_pos, &log);
  }
}

void tiltDown() {
  dxl_wb.goalPosition(5, tilt_home);
}

void scoopUp() {
  dxl_wb.goalPosition(7, scoop_home_7 + 1175);
  dxl_wb.goalPosition(8, scoop_home_8 - 1175);
}
void scoopDown() {
  dxl_wb.goalPosition(7, scoop_home_7);
  dxl_wb.goalPosition(8, scoop_home_8);
}
void dumpUp() {
  dxl_wb.goalPosition(10, dump_1_home - 1250);
}
void dumpDown(){
  dxl_wb.goalPosition(10, dump_1_home);
}