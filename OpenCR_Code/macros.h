#define DXL_FR_ID 1          // ID of the front-right Dynamixel
#define DXL_FL_ID 2          // ID of the front-left Dynamixel
#define DXL_RR_ID 3          // ID of the rear-right Dynamixel
#define DXL_RL_ID 4          // ID of the rear-left Dynamixel
#define DXL_TILT 5           // ID of tilt Dynamixel
#define DXL_SORT 6           // ID of sort Dynamixel
#define DXL_SCOOP_R 7          // ID of first collection Dynamixel
#define DXL_SCOOP_L 8          // ID of second collection Dynamixel
#define DXL_DUMP_L 9
#define DXL_DUMP_R 10

#define HOMEING_VEL \
  { 0, 0, 0, 0, 0, 50, 0, 50, 50, 50, 50}

#define ACCELERATION_MODE 0  // Argument for wheel mode
#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3
#define EXT_POSITION_CONTROL_MODE 4
#define POSITION_TRESHOLD 50
#define TIMEOUT 5000 //5 Second timeout
#define STOP  0
#define HOMING_VELOCITY 50  // Velocity of Homing function
#define LOAD_THRESHOLD 150  // Value of load to complete the homing task
#define MOTOR_IDS { 1, 2, 3, 4 }
#define SAMPLING_PERIOD 50   // Sampling period in microseconds
#define BAUDRATE 57600       // Baudrate of Dynamixel communication

#if defined(__OPENCM904__)
#define DEVICE_NAME "DXL_PORT"
#elif defined(__OPENCR__)
#define DEVICE_NAME "DXL_PORT"
#endif
#define TILT_UP_POS 2200
#define TILT_DOWN_POS 2840
#define SCOOP_R_DOWN_POS 3075
#define SCOOP_UP_POS 1100
#define SCOOP_DOWN_POS	0
#define SCOOP_L_DOWN_POS 2025
#define SCOOP_L_UP_POS 900
#define DUMP_L_DOWN_POS 492
#define DUMP_L_UP_POS -700
#define DUMP_R_DOWN_POS 2018
#define DUMP_R_UP_POS 900
#define SORT_CENTER 1050
#define SORT_NEB	-750
#define SORT_GEO	2875
//States



#define NUMBER_OF_MOTORS 6               // Total number of motors
#define POSITION_THRESHOLD 20             // Adjust based on your motor's precision
#define QUEUE_SIZE 20                     // Max number of positions in each motor's queue

extern bool scoop_request;
extern bool scoop_done;
extern bool sort_done;


#include <stdint.h>
#include <DynamixelWorkbench.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Char.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <IMU.h>

extern DynamixelWorkbench dxl_wb;
extern ros::NodeHandle nh;