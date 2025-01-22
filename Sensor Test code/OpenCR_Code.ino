
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <IMU.h>
#include <DynamixelWorkbench.h>
#include <math.h>

#if defined(__OPENCM904__)
  #define DEVICE_NAME "DXL_PORT" // Dynamixel on Serial3(USART3) <- OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME "DXL_PORT"
#endif 

// MACROS
#define SAMPLING_PERIOD 50 // Sampling period in microseconds
#define BAUDRATE 57600 // Baudrate of Dynamixel communication
#define DXL_FR_ID 1 // ID of the front-right Dynamixel
#define DXL_FL_ID 2 // ID of the front-left Dynamixel
#define DXL_RR_ID 3 // ID of the rear-right Dynamixel
#define DXL_RL_ID 4 // ID of the rear-left Dynamixel
#define DXL_TROUGH_ID 5 //ID of trough Motor
#define DXL_SORT_ID 6 //ID of sorting motor 
#define ACCELERATION_MODE 1 // Argument for wheel mode
#define VELOCITY_CONTROL_MODE 1
#define FORWARD_VELOCITY 1.0f  // Set a higher velocity
#define MOTOR_IDS {1, 2, 3, 4, 5, 6}



uint8_t dxl_FL = DXL_FL_ID;
uint8_t dxl_FR = DXL_FR_ID;
uint8_t dxl_RL = DXL_RL_ID;
uint8_t dxl_RR = DXL_RR_ID;
uint8_t dxl_trough = DXL_TROUGH_ID;
uint8_t dxl_sort = DXL_SORT_ID;
int32_t trough_home_speed = 0.1;    //sets the speed of the trough motor while homing
int32_t troughStop = 0.0;
float troughHome = 0.0;   //trough home position
// ROS node handle
ros::NodeHandle nh;

// Dynamixel Workbench instance
DynamixelWorkbench dxl_wb;

// Encoder data publisher
std_msgs::Float64MultiArray encoder_data;
ros::Publisher encoder_pub("/wheel_encoders", &encoder_data);

bool result = false;

void wheel_1_write(const std_msgs::Float64& msg)
{
  dxl_wb.goalVelocity(dxl_FR, (int32_t)msg.data);
}
void wheel_2_write(const std_msgs::Float64& msg)
{
  dxl_wb.goalVelocity(dxl_FL, (int32_t)msg.data);
}
void wheel_3_write(const std_msgs::Float64& msg)
{
  dxl_wb.goalVelocity(dxl_RR, (int32_t)msg.data);
}
void wheel_4_write(const std_msgs::Float64& msg)
{
  dxl_wb.goalVelocity(dxl_RL, (int32_t)msg.data);
}
void trough_write(const std_msgs::Float64& msg){
  dxl_wb.goalPosition(dxl_trough, (int32_t)msg.data);
}

// Wheel speed subscribers

ros::Subscriber<std_msgs::Float64> w1_topic("/wheel_1_speed", wheel_1_write );
ros::Subscriber<std_msgs::Float64> w2_topic("/wheel_2_speed", wheel_2_write );
ros::Subscriber<std_msgs::Float64> w3_topic("/wheel_3_speed", wheel_3_write );
ros::Subscriber<std_msgs::Float64> w4_topic("/wheel_4_speed", wheel_4_write );
ros::Subscriber<std_msgs::Float64> trough_topic("/trough_speed", trough_write );


// Publish encoder data
void publishEncoderData() {
  encoder_data.data_length = 4; // Ensure array has the correct length
  float radian; // Temporary variable to store radian values

  if (dxl_wb.getRadian(1, &radian)) {
    encoder_data.data[0] = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 1");
    encoder_data.data[0] = 0.0;
  }

  if (dxl_wb.getRadian(2, &radian)) {
    encoder_data.data[1] = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 2");
    encoder_data.data[1] = 0.0;
  }

  if (dxl_wb.getRadian(3, &radian)) {
    encoder_data.data[2] = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 3");
    encoder_data.data[2] = 0.0;
  }

  if (dxl_wb.getRadian(4, &radian)) {
    encoder_data.data[3] = radian;
  } else {
    nh.logerror("Failed to get encoder data for motor 4");
    encoder_data.data[3] = 0.0;
  }
  encoder_pub.publish(&encoder_data);
}


void setup()
{
  Serial.begin(57600);
  nh.subscribe(w1_topic);
  nh.subscribe(w2_topic);
  nh.subscribe(w3_topic);
  nh.subscribe(w4_topic);
  nh.advertise(encoder_pub);
  const char *log;
  bool result;
  
  // Initialize Dynamixel
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log);
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
  for (int i = 0; i < 6; i++) {
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

    // Set the motor to Velocity Control Mode
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

  dxl_wb.goalVelocity(dxl_trough, trough_home_speed); //trough begins lowering slowly
  int32_t currentLoad = 0;
  if(dxl_wb.itemRead(dxl_trough, "Present_Load", &currentLoad)){ //checks current motor load
    float load_percentage = (abs(currentLoad) / 1023.0) * 100.0;
    if(currentLoad < 0 && load_percentage > 20){   
      dxl_wb.goalVelocity(dxl_trough, troughStop);   //if load is negative (clockwise), and greater than 20% max capacity, stop
    }
  }
  
  dxl_wb.setPositionControlMode(dxl_trough);  //change trough to position control
  dxl_wb.getRadian(dxl_trough, &troughHome);  //save the radian value of home location
}

// Main loop function
void loop() {
  nh.spinOnce();
  publishEncoderData(); // Publish encoder feedback
  delay(SAMPLING_PERIOD);
}
