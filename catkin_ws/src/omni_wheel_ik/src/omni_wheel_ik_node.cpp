#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>
#include <cmath>


// Robot-specific constants
const float WHEEL_RADIUS = 0.056; // Wheel radius in meters
const float ROBOT_RADIUS = 0.1425; // Distance from the center to the wheels in meters

ros::Publisher wheel_1;
ros::Publisher wheel_2;
ros::Publisher wheel_3;
ros::Publisher wheel_4;

std_msgs::Float64 wheel1_speed;
std_msgs::Float64 wheel2_speed;
std_msgs::Float64 wheel3_speed;
std_msgs::Float64 wheel4_speed;
void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg)
{
    // Extract desired velocities
    float vx = msg->x; // Linear velocity in x direction (m/s)
    float vy = msg->y; // Linear velocity in y direction (m/s)
    float omega = msg->z; // Angular velocity around z-axis (rad/s)

    // Robot heading angle (assuming theta = 0 for simplicity; replace with actual theta if available)
    float theta = 0;

    // Precompute trigonometric values
    float sin_theta_pi_4 = std::sin(theta + M_PI / 4);
    float cos_theta_pi_4 = std::cos(theta + M_PI / 4);
    float sin_theta_3pi_4 = std::sin(theta + 3 * M_PI / 4);
    float cos_theta_3pi_4 = std::cos(theta + 3 * M_PI / 4);
    float sin_theta_5pi_4 = std::sin(theta + 5 * M_PI / 4);
    float cos_theta_5pi_4 = std::cos(theta + 5 * M_PI / 4);
    float sin_theta_7pi_4 = std::sin(theta + 7 * M_PI / 4);
    float cos_theta_7pi_4 = std::cos(theta + 7 * M_PI / 4);

    // Calculate wheel speeds using inverse kinematics
    wheel1_speed.data = -(1 / WHEEL_RADIUS) * (-sin_theta_pi_4 * vx + cos_theta_pi_4 * vy + ROBOT_RADIUS * omega);
    wheel2_speed.data = -(1 / WHEEL_RADIUS) * (-sin_theta_3pi_4 * vx + cos_theta_3pi_4 * vy + ROBOT_RADIUS * omega);
    wheel3_speed.data = -(1 / WHEEL_RADIUS) * (-sin_theta_5pi_4 * vx + cos_theta_5pi_4 * vy + ROBOT_RADIUS * omega);
    wheel4_speed.data = -(1 / WHEEL_RADIUS) * (-sin_theta_7pi_4 * vx + cos_theta_7pi_4 * vy + ROBOT_RADIUS * omega);

    // Prepare the message
   // std_msgs::Float64MultiArray wheel_speeds_msg;
   // wheel_speeds_msg.data = {wheel1_speed, wheel2_speed, wheel3_speed, wheel4_speed};

    // Publish the wheel speeds
    wheel_1.publish(wheel1_speed);
    wheel_2.publish(wheel2_speed);
    wheel_3.publish(wheel3_speed);
    wheel_4.publish(wheel4_speed);

}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "omni_wheel_ik_node");
    ros::NodeHandle nh;

    // Publisher for wheel speeds
    wheel_1 = nh.advertise<std_msgs::Float64>("wheel_1_speed", 10);
    wheel_2 = nh.advertise<std_msgs::Float64>("wheel_2_speed", 10);
    wheel_3 = nh.advertise<std_msgs::Float64>("wheel_3_speed", 10);
    wheel_4 = nh.advertise<std_msgs::Float64>("wheel_4_speed", 10);

    // Subscriber for desired velocities
    ros::Subscriber velocity_sub = nh.subscribe("local_velocities", 10, velocityCallback);
    // Spin to process callbacks
    ros::spin();

    return 0;
}
