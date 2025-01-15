#include <ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define TOLERANCE 0.01  // Tolerance for reaching the target position
#define MOTOR_SPEED 1.0 // Default motor speed in rad/s
#define Home 0.0  //Encoder home, will likely need to seperate for Trough and Sort

// Global variables for encoder values
double sort_current_pos = 0.0;
double trough_current_pos = 0.0;
double target = 0.0;

std_msgs::Float64 command;
std_msgs::Float64 STOP;

ros::Rate rate(100);  // 100 Hz loop rate

// Callback for Sort motor encoder
void SortEncoderCallback(const std_msgs::Float64::ConstPtr& msg) {
    sort_current_pos = msg->data;
}

// Callback for Trough motor encoder
void TroughEncoderCallback(const std_msgs::Float64::ConstPtr& msg) {
    trough_current_pos = msg->data;
}

void Rotate(double CURR, double TARGET){
  while (fabs(CURR - TARGET) > TOLERANCE) {
        command.data = MOTOR_SPEED * ((CURR < TARGET) ? 1 : -1);
        pub.publish(command);
        ros::spinOnce();
        rate.sleep();
    }
  pub.publish(STOP); // Stop motor
}

//Sort one direction and reset
void MotorControl(ros::Publisher& pub, target) {
    Rotate(sort_current_pos, target);  // Move to π/3
    Rotate(sort_current_pos, 0); // Return to 0
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;

    // Publishers for motor angular velocity
    ros::Publisher sort_pub = nh.advertise<std_msgs::Float64>"/6_ang_vel", 10);
    ros::Publisher trough_pub = nh.advertise<std_msgs::Float64>"/5_ang_vel", 10);

    // Subscribers for encoders
    ros::Subscriber sort_enc_sub = nh.subscribe("/6_enc", 10, SortEncoderCallback);
    ros::Subscriber trough_enc_sub = nh.subscribe("/5_enc", 10, TroughEncoderCallback);

    // Example usage of the functions
    target = M_PI / 5;
    MotorControl(trough_pub, target); //Spill Trough
    target = 2 * M_PI / 3;
    MotorControl(sort_pub, target); //Sort one way
    target = -2 * M_PI / 3;
    MotorControl(sort_pub, target);
    return 0;
}
