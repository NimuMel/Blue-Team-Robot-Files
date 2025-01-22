#define DXL_FR_ID 1          // ID of the front-right Dynamixel
#define DXL_FL_ID 2          // ID of the front-left Dynamixel
#define DXL_RR_ID 3          // ID of the rear-right Dynamixel
#define DXL_RL_ID 4          // ID of the rear-left Dynamixel
#define DXL_TILT 5           // ID of tilt Dynamixel
#define DXL_SORT 6           // ID of sort Dynamixel
#define DXL_COL_1 7          // ID of first collection Dynamixel
#define DXL_COL_1 8          // ID of second collection Dynamixel
#define ACCELERATION_MODE 0  // Argument for wheel mode
#define VELOCITY_CONTROL_MODE 1
#define POSITION_CONTROL_MODE 3
#define EXT_POSITION_CONTROL_MODE 4
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


#include <stdint.h> // For uint8_t, uint16_t
#include <DynamixelWorkbench.h>
#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <IMU.h>