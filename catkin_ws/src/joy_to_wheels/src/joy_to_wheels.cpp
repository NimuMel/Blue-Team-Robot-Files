#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>  // For (vx, vy, omega)
#include <std_msgs/Float32.h>       // For individual wheel speeds
#include <sensor_msgs/Joy.h>        // For joystick inputs

// Robot parameters
const double RADIUS = 0.05;  // Wheel radius (meters)
const double L = 0.2;        // Distance from robot center to wheels (meters)

// Inverse Kinematics Matrix (for 45-degree offset wheels)
const double IK_MATRIX[4][3] = {
    {  1.0 / RADIUS, -1.0 / RADIUS, -L / RADIUS },  // Wheel 1
    {  1.0 / RADIUS,  1.0 / RADIUS,  L / RADIUS },  // Wheel 2
    { -1.0 / RADIUS,  1.0 / RADIUS, -L / RADIUS },  // Wheel 3
    { -1.0 / RADIUS, -1.0 / RADIUS,  L / RADIUS }   // Wheel 4
};

// Publishers for wheel velocities
ros::Publisher pub_wheel1;
ros::Publisher pub_wheel2;
ros::Publisher pub_wheel3;
ros::Publisher pub_wheel4;

// Variables for joystick inputs
double max_linear_speed = 1.0;  // Max linear speed (m/s)
double max_angular_speed = 1.0; // Max angular speed (rad/s)

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg) {
    // Joystick axes typically range from -1 to 1
    // Axis 1 for forward/backward (vx), Axis 0 for left/right (vy), Axis 3 for rotation (omega)

    double vx = joy_msg->axes[1] * max_linear_speed; // Forward/backward speed
    double vy = joy_msg->axes[0] * max_linear_speed; // Left/right speed
    double omega = joy_msg->axes[3] * max_angular_speed; // Rotation speed

    // Compute wheel speeds
    double wheel_speeds[4];
    for (int i = 0; i < 4; ++i) {
        wheel_speeds[i] = IK_MATRIX[i][0] * vx +
                          IK_MATRIX[i][1] * vy +
                          IK_MATRIX[i][2] * omega;
    }

    // Publish wheel speeds
    std_msgs::Float32 wheel1_msg, wheel2_msg, wheel3_msg, wheel4_msg;
    wheel1_msg.data = wheel_speeds[0];
    wheel2_msg.data = wheel_speeds[1];
    wheel3_msg.data = wheel_speeds[2];
    wheel4_msg.data = wheel_speeds[3];

    pub_wheel1.publish(wheel1_msg);
    pub_wheel2.publish(wheel2_msg);
    pub_wheel3.publish(wheel3_msg);
    pub_wheel4.publish(wheel4_msg);
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "velocity_to_wheels_with_joy");
    ros::NodeHandle nh;

    // Initialize publishers
    pub_wheel1 = nh.advertise<std_msgs::Float32>("wheel1_speed", 10);
    pub_wheel2 = nh.advertise<std_msgs::Float32>("wheel2_speed", 10);
    pub_wheel3 = nh.advertise<std_msgs::Float32>("wheel3_speed", 10);
    pub_wheel4 = nh.advertise<std_msgs::Float32>("wheel4_speed", 10);

    // Subscribe to the /joy topic to receive joystick input
    ros::Subscriber sub_joy = nh.subscribe("joy", 10, joyCallback);

    // Spin to process callbacks
    ros::spin();

    return 0;
}
