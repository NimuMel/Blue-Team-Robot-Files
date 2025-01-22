#include <ros.h>
#include <std_msgs/Float64.h>
#include <math.h>

#define TOLERANCE 0.01  // Tolerance for reaching the target position
#define MOTOR_SPEED 1.0 // Default motor speed in rad/s
#define SortHome 0.0  //Sorting Encoder home
#define TroughHome 0.0 //Trough Encoder home
#define BAUDRATE 57600 // Baudrate of Dynamixel communication


// Global variables for encoder values
double sort_current_pos = 0.0;
double trough_current_pos = 0.0;
double target = 0.0;
double speed;

std_msgs::Float64 command;
std_msgs::Float64 STOP;

// Callback for Sort motor encoder
void SortEncoderCallback(const std_msgs::Float64::Const& msg) {
    sort_current_pos = msg->data;
}

// Callback for Trough motor encoder
void TroughEncoderCallback(const std_msgs::Float64::Const& msg) {
    trough_current_pos = msg->data;
}

void Rotate(double CURR, double TARGET, ros::Publisher& pub, double speed){
  //Rotate until Target is reached
  while (fabs(CURR - TARGET) > TOLERANCE) {
        command.data = speed * ((CURR < TARGET) ? 1 : -1);
        pub.publish(command);
        ros::spinOnce();
    }
  pub.publish(STOP); // Stop motor
}

//Sort one direction and reset
void SortMotorControl(ros::Publisher& pub, double target) {
    Rotate(sort_current_pos, target, pub);  // Move to target
    Rotate(sort_current_pos, SortHome, pub); // Return to home
}
void TroughMotorControl(ros::Publisher& pub, double target) {
    Rotate(trough_current_pos, target, pub);  // Move to target
    Rotate(trough_current_pos, TroughHome, pub); // Return to home
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "motor_controller");
    ros::NodeHandle nh;

    STOP.data = 0.0;

    // Publishers for motor angular velocity
    ros::Publisher sort_pub = nh.advertise("/6_ang_vel", 10);
    ros::Publisher trough_pub = nh.advertise("/5_ang_vel", 10);

    // Subscribers for encoders
    ros::Subscriber<std_msgs::Float64> sort_enc_sub = nh.subscribe("/6_enc", 10, SortEncoderCallback);
    ros::Subscriber<std_msgs::Float64> trough_enc_sub = nh.subscribe("/5_enc", 10, TroughEncoderCallback);

    speed = 0.5;
    // Example usage of the functions
    TroughMotorControl(trough_pub, M_PI / 5, speed); //Spill Trough
    SortMotorControl(sort_pub, 2 * M_PI / 3, speed); //Sort one way
    SortMotorControl(sort_pub, -2 * M_PI / 3, speed);
    return 0;
}
