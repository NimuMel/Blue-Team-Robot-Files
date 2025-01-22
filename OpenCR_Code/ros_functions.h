
#include "macros.h"
void ros_inti();

void wheel_1_write(const std_msgs::Float64& msg);
void wheel_2_write(const std_msgs::Float64& msg);
void wheel_3_write(const std_msgs::Float64& msg);
void wheel_4_write(const std_msgs::Float64& msg);

void publishMotor1Data();
void publishMotor2Data();
void publishMotor3Data();
void publishMotor4Data();