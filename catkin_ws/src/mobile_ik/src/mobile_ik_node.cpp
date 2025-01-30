#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

const double wheel_radius = 0.0508; // Wheel radius (meters)
const double wheel_base = 0.3;    // Distance between the wheels (meters)

ros::Publisher right_wheel_ang_vel_pub;
ros::Publisher left_wheel_ang_vel_pub;

void velocityCallback(const geometry_msgs::Vector3::ConstPtr& msg) {
    double dot_x = msg->x;  // Desired linear velocity in x
    double dot_y = 0;  // Desired linear velocity in y (should be 0 for differential drive)
    double dot_theta = msg->z * 3.14;  // Desired angular velocity

    // Calculate wheel velocities using the inverse Jacobian
    double v_left = (dot_x - (wheel_base / 2) * dot_theta) / wheel_radius;
    double v_right = (dot_x + (wheel_base / 2) * dot_theta) / wheel_radius;

    // Create messages
    std_msgs::Float64 left_msg;
    std_msgs::Float64 right_msg;

    left_msg.data = v_left;
    right_msg.data = v_right;

    // Publish the calculated wheel velocities
    left_wheel_ang_vel_pub.publish(left_msg);
    right_wheel_ang_vel_pub.publish(right_msg);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "mobile_ik_node");
    ros::NodeHandle nh;

    right_wheel_ang_vel_pub = nh.advertise<std_msgs::Float64>("right_wheel_ang_vel", 10);
    left_wheel_ang_vel_pub = nh.advertise<std_msgs::Float64>("left_wheel_ang_vel", 10);

    ros::Subscriber vel_sub = nh.subscribe("local_velocities", 10, velocityCallback);

    ros::spin();
    return 0;
}
