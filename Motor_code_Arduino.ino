#include <ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define TOLERANCE 0.01  // Tolerance for reaching the target position
#define MOTOR_SPEED 1.0 // Default motor speed in rad/s
#define BAUDRATE 57600  // Baudrate for ROS communication
#define Home 0.0        // Encoder home position

// Global variables
double sort_current_pos = 0.0;
double trough_current_pos = 0.0;
std_msgs::Float64 command;
std_msgs::Float64 STOP;

ros::NodeHandle nh;

// Publishers
ros::Publisher sort_pub("/6_ang_vel", &command);
ros::Publisher trough_pub("/5_ang_vel", &command);

// Callbacks
void SortEncoderCallback(const std_msgs::Float64& msg) {
    sort_current_pos = msg.data;
}

void TroughEncoderCallback(const std_msgs::Float64& msg) {
    trough_current_pos = msg.data;
}

ros::Subscriber<std_msgs::Float64> sort_enc_sub("/6_enc", &SortEncoderCallback);
ros::Subscriber<std_msgs::Float64> trough_enc_sub("/5_enc", &TroughEncoderCallback);

void Rotate(double CURR, double TARGET, ros::Publisher& pub) {
    while (fabs(CURR - TARGET) > TOLERANCE) {
        command.data = MOTOR_SPEED * ((CURR < TARGET) ? 1 : -1);
        pub.publish(&command);
        nh.spinOnce();
        delay(10); // 100 Hz
    }
    pub.publish(&STOP);
}

void MotorControl(ros::Publisher& pub, double target) {
    Rotate(sort_current_pos, target, pub);
    Rotate(sort_current_pos, Home, pub);
}

void setup() {
    nh.getHardware()->setBaud(BAUDRATE);
    nh.initNode();

    nh.advertise(sort_pub);
    nh.advertise(trough_pub);
    nh.subscribe(sort_enc_sub);
    nh.subscribe(trough_enc_sub);

    STOP.data = 0.0;
}

void loop() {
    nh.spinOnce();
    MotorControl(trough_pub, M_PI / 5);
    MotorControl(sort_pub, 2 * M_PI / 3);
    MotorControl(sort_pub, -2 * M_PI / 3);
    delay(1000);
}
