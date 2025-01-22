#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Vector3.h>

// Publisher for local velocities
ros::Publisher local_vel_pub;

// Callback to process Xbox controller inputs
void joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
    geometry_msgs::Vector3 local_vel;

    // Left joystick controls: axes[1] (forward/backward), axes[0] (left/right)
    local_vel.x = joy->axes[1];  // Forward/Backward
    local_vel.z = joy->axes[0];  // Left/Right
    local_vel.y = joy->axes[3];  // Right joystick x-axis for rotation

    local_vel_pub.publish(local_vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "xbox_controller_node");
    ros::NodeHandle nh;

    // Subscribe to the /joy topic
    ros::Subscriber joy_sub = nh.subscribe<sensor_msgs::Joy>("/joy", 10, joyCallback);

    // Publish to the local_velocities topic
    local_vel_pub = nh.advertise<geometry_msgs::Vector3>("local_velocities", 10);

    ros::spin();
    return 0;
}
