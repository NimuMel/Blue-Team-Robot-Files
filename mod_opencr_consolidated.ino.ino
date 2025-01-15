/*******************************************************************************

*******************************************************************************/

/* Authors: Dr. Eli Buckner, North Carolina State University */

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <IMU.h>
#include <DynamixelWorkbench.h>
#include <std_msgs/UInt16.h>
#include <std_msgs/Bool.h>

#define HEPIN 9
#define BUTTON0 2
#define BUTTON1 3
#define BUTTON2 4
#define BUTTON3 5
#define LIGHT_SENS 300

ros::NodeHandle swHandle;
std_msgs::Bool buttonState0, buttonState1, buttonState2, buttonState3, ledDetect, b0prev, b1prev, b2prev, b3prev, ledPrev, heState;

#if defined(__OPENCM904__)
  #define DEVICE_NAME "3" //Dynamixel on Serial3(USART3)  <-OpenCM 485EXP
#elif defined(__OPENCR__)
  #define DEVICE_NAME ""
#endif 

// MACROS
#define SAMPLING_PERIOD                   50 // Sampling period in microseconds
#define BAUDRATE                          57600 // Baudrate of Dynamixel communication
#define DXL_RT_ID                         1 // ID of the left Dynamixel
#define DXL_LF_ID                         2 // ID of the right Dynamixel
#define ACCELERATION_MODE                 0 // Argument for wheel mode
#define RW_INITIAL_CONDITION              0.0 // Initial wheel velocity of right wheel at startup
#define LW_INITIAL_CONDITION              0.0 // Initial wheel velocity of the left wheel at startup

ros::NodeHandle nh; // Node handle object

// Message types for subscription and publication of data.
sensor_msgs::Imu imu_msg;
geometry_msgs::TransformStamped tfs_msg;
tf::TransformBroadcaster tfbroadcaster;
std_msgs::Float64 right_wheel_ang_vel;
std_msgs::Float64 right_wheel_enc_msg;
std_msgs::Float64 left_wheel_ang_vel;
std_msgs::Float64 left_wheel_enc_msg;
std_msgs::Float64 z_degrees;    //LOGAN WROTE THIS; this value will be published by the z_deg topic and will take the z orientation and multiply it by 180
std_msgs::Float64 z_deg_prev;
std_msgs::Float64 z_diff;
float* right_radians;
float* left_radians;

// Initiate all publications.
ros::Publisher imu_pub("imu", &imu_msg);
ros::Publisher z_orientation("z_deg", &z_degrees);    //LOGAN WROTE THIS; this creates the publisher object
ros::Publisher right_wheel_enc_pub("right_wheel_enc", &right_wheel_enc_msg);
ros::Publisher left_wheel_enc_pub("left_wheel_enc", &left_wheel_enc_msg);
ros::Publisher b0Pub("b0", &buttonState0);
ros::Publisher b1Pub("b1", &buttonState1);
ros::Publisher b2Pub("b2", &buttonState2);
ros::Publisher b3Pub("b3", &buttonState3);
ros::Publisher prPub("led_det", &ledDetect);
ros::Publisher hePub("hall_det", &heState);

// Global objects
cIMU imu; // Create an imu object
DynamixelWorkbench dxl_wb; // Create a dynamixel workbench object

// Global variables
bool result = false;
uint8_t dxl_lf_id = DXL_LF_ID;
uint8_t dxl_rt_id = DXL_RT_ID;
uint16_t model_number = 0;

/*
Callback function for when data is written to the "right_wheel_ang_vel" topic. This topic takes the 
data as a floating point number that is the value of the angular velocity to be written to the 
Dynamixel motor controlling the right wheel and stores it to a global variable.
*/
void right_wheel_write(const std_msgs::Float64& msg)
{
  right_wheel_ang_vel.data = msg.data;
}

/*
Callback function for when data is written to the "left_wheel_ang_vel" topic. This topic takes the 
data as a floating point number that is the value of the angular velocity to be written to the 
Dynamixel motor controlling the left wheel and stores it to a global variable.
*/
void left_wheel_write(const std_msgs::Float64& msg)
{
  left_wheel_ang_vel.data = msg.data;
}

// Initiate all subscriptions
ros::Subscriber<std_msgs::Float64> rw_topic("right_wheel_ang_vel", right_wheel_write );
ros::Subscriber<std_msgs::Float64> lw_topic("left_wheel_ang_vel", left_wheel_write );

void setup()
{
  Serial.begin(57600);
  // Initial conditions
  right_wheel_ang_vel.data = RW_INITIAL_CONDITION;
  left_wheel_ang_vel.data = LW_INITIAL_CONDITION;
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(HEPIN, INPUT);

  swHandle.initNode();
  swHandle.advertise(b0Pub);
  swHandle.advertise(b1Pub);
  swHandle.advertise(b2Pub);
  swHandle.advertise(b3Pub);
  swHandle.advertise(prPub);
  swHandle.advertise(hePub);
  const char *log; // Holds a printout log for the init function
  result = dxl_wb.init(DEVICE_NAME, BAUDRATE, &log); // Initialize the workbench
  if (result == false) // Make sure that the initialization was successful
  {
    Serial.println(log);
    Serial.println("Failed to init");
  }
  else
  {
    Serial.print("Succeeded to init : ");
    Serial.println(BAUDRATE);  
  }

  result = dxl_wb.ping(dxl_lf_id, &model_number, &log); // Ping the left wheel
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping left wheel");
    Serial.print("id : ");
    Serial.print(dxl_lf_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }

  result = dxl_wb.ping(dxl_rt_id, &model_number, &log); // Ping the right wheel
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to ping");
  }
  else
  {
    Serial.println("Succeeded to ping right wheel");
    Serial.print("id : ");
    Serial.print(dxl_rt_id);
    Serial.print(" model_number : ");
    Serial.println(model_number);
  }

  result = dxl_wb.wheelMode(dxl_lf_id, ACCELERATION_MODE, &log); // Set the wheel mode 
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to change wheel mode");
  }
  else
  {
    Serial.println("Left wheel mode set successfully");
  }

  result = dxl_wb.wheelMode(dxl_rt_id, ACCELERATION_MODE, &log); // Set the wheel mode 
  if (result == false)
  {
    Serial.println(log);
    Serial.println("Failed to change wheel mode");
  }
  else
  {
    Serial.println("Right wheel mode set successfully");
  }

  nh.initNode(); // Launch node

  nh.advertise(z_orientation);
  nh.advertise(imu_pub); // Launch topics
  nh.advertise(right_wheel_enc_pub);
  nh.advertise(left_wheel_enc_pub);
  nh.subscribe(rw_topic);
  nh.subscribe(lw_topic);
  tfbroadcaster.init(nh);

  imu.begin(); // Launch imu
}

void loop()
{
  static uint32_t pre_time; // Used to give constant sampling period

  imu.update(); // Take sample from the IMU

  if (millis()-pre_time >= SAMPLING_PERIOD)
  {
    pre_time = millis();

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

    if(imu.gyroData[2] > 0){      //LOGAN WROTE THIS basically if the orientation is decreasing while rotation occurs in the positive direction it corrects, same if orientation increases with negative rotational velocity.
      if(imu.quat[3] < z_deg_prev.data){
        z_degrees.data = imu.quat[3] * 180;
      }
      else if(imu.quat[3] > z_deg_prev.data){
        z_degrees.data = imu.quat[3] * -180;
      }
    }
    if(imu.gyroData[2] < 0){
      if(imu.quat[3] > z_deg_prev.data){
        z_degrees.data = imu.quat[3] * 180;
      }
      else if(imu.quat[3] < z_deg_prev.data){
        z_degrees.data = imu.quat[3] * -180;
      }
    }

    imu_msg.orientation_covariance[0] = 0.0025;
    imu_msg.orientation_covariance[1] = 0;
    imu_msg.orientation_covariance[2] = 0;
    imu_msg.orientation_covariance[3] = 0;
    imu_msg.orientation_covariance[4] = 0.0025;
    imu_msg.orientation_covariance[5] = 0;
    imu_msg.orientation_covariance[6] = 0;
    imu_msg.orientation_covariance[7] = 0;
    imu_msg.orientation_covariance[8] = 0.0025;

    dxl_wb.getRadian(dxl_rt_id, right_radians); // Get the position of the motors in radians
    right_wheel_enc_msg.data = *right_radians;
    Serial.print("Right radians = ");
    Serial.println(*right_radians);
    dxl_wb.getRadian(dxl_lf_id, left_radians); // Get the position of the motors in radians
    Serial.println("Received radian measure from Dynamixel");
    left_wheel_enc_msg.data = *left_radians;
    Serial.print("Left radians = ");
    Serial.println(*left_radians);
    z_orientation.publish(&z_degrees);    //LOGAN WROTE THIS; publishes the z_degrees number which is just z_orientation times 180
    z_deg_prev.data = imu.quat[3];     //SAVE CURRENT ORIENTATION TO CHECK AGAINST NEW ORIENTATION

    imu_pub.publish(&imu_msg); // Publish
    Serial.println("IMU published");
    right_wheel_enc_pub.publish(&right_wheel_enc_msg);
    left_wheel_enc_pub.publish(&left_wheel_enc_msg);
    Serial.println("Data published");

    tfs_msg.header.stamp    = nh.now();
    tfs_msg.header.frame_id = "base_link";
    tfs_msg.child_frame_id  = "imu_link";
    tfs_msg.transform.rotation.w = imu.quat[0];
    tfs_msg.transform.rotation.x = imu.quat[1];
    tfs_msg.transform.rotation.y = imu.quat[2];
    tfs_msg.transform.rotation.z = imu.quat[3];

    tfs_msg.transform.translation.x = 0.0;
    tfs_msg.transform.translation.y = 0.0;
    tfs_msg.transform.translation.z = 0.0;

    tfbroadcaster.sendTransform(tfs_msg);

    dxl_wb.goalVelocity(dxl_lf_id, (float)left_wheel_ang_vel.data); // Set velocity for wheels
    dxl_wb.goalVelocity(dxl_rt_id, (float)right_wheel_ang_vel.data);


  //check light val
  int light = analogRead(A0);
    heState.data = !digitalRead(HEPIN);
    hePub.publish(&heState);
    //Serial output demonstrates difference between buttons
    
  bool CONDITION = false;
  std::string LABEL="button 0";
  uint16_t NUMBUTTONS = 4;
  std_msgs::Bool PREV, CURR;
  ros::Publisher* PUB = nullptr;
  for(int i=0;i<NUMBUTTONS;i++){

switch(i){
  case 0:
		PREV = b0prev;
		CURR = buttonState0;
		LABEL = "button 0";
		CONDITION = (digitalRead(BUTTON0) == HIGH);
    PUB = &b0Pub;
    break;
	case 1:
		PREV = b1prev;
		CURR = buttonState1;
		LABEL = "button 1";
		CONDITION = (digitalRead(BUTTON1) == HIGH);
    PUB = &b1Pub;
    break;
  case 2:	
		PREV = b2prev;
		CURR = buttonState2;
		LABEL = "button 2";
		CONDITION = (digitalRead(BUTTON2) == HIGH);
    PUB = &b2Pub;
    break;
	case 3:
		PREV = b3prev;
		CURR = buttonState3;
		LABEL = "button 3";
		CONDITION = (digitalRead(BUTTON3) == HIGH);
    PUB = &b3Pub;
    break;
	case 4:
		PREV = b3prev;
		CURR = buttonState3;
		LABEL = "LED";
		CONDITION = (light > LIGHT_SENS);
    PUB = &prPub;
    break;
}

	PREV = CURR;
      if (CONDITION){
        Serial.println(LABEL.c_str());
        CURR.data = true;
      } else{
        CURR.data = false;
      }
      if(PREV.data != CURR.data){
        PUB->publish(&CURR );
      }
}

  }

  nh.spinOnce();
}
