
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Pose2D.h>
#include <vector>
#include <unistd.h>

// Robot-specific constants
const float WHEEL_RADIUS = 0.056; // Wheel radius in meters
const float ROBOT_RADIUS = 0.27; // Robot radius in meters
const float TWO_PI = 2.0 * M_PI;
const float DT = 0.02; // Time step (assumes 100 Hz update rate)

// Global pose
geometry_msgs::Pose2D robot_pose;
std::vector<double> previous_encoder_values(4, 0.0);

// Current encoder values (global to be updated by callbacks)
std::vector<double> current_encoder_values(4, 0.0);

bool is_initialized = false;

// Callback functions for each encoder
void encoder1Callback(const std_msgs::Float32::ConstPtr& msg)
{
    current_encoder_values[0] = msg->data;
}

void encoder2Callback(const std_msgs::Float32::ConstPtr& msg)
{
    current_encoder_values[1] = msg->data;
}

void encoder3Callback(const std_msgs::Float32::ConstPtr& msg)
{
    current_encoder_values[2] = msg->data;
}

void encoder4Callback(const std_msgs::Float32::ConstPtr& msg)
{
    current_encoder_values[3] = msg->data;
}

void calculatePose()
{
    if (!is_initialized)
    {
        previous_encoder_values = current_encoder_values;
        is_initialized = true;
        ROS_INFO("Pose estimation initialized with encoder values.");
    }

    // Calculate wheel angular velocities from encoder values
    std::vector<double> wheel_speeds(4, 0.0);
    for (size_t i = 0; i < 4; ++i)
    {
        wheel_speeds[i] = (current_encoder_values[i] - previous_encoder_values[i]) / DT;
    }

    // Update previous encoder values
    previous_encoder_values = current_encoder_values;

    // Robot's current heading
    double theta = robot_pose.theta;

    // Jacobian matrix elements for forward kinematics
    double J[4][3] = {
        {-sin(theta + M_PI / 4), cos(theta + M_PI / 4), 1 / (2 * ROBOT_RADIUS)},
        {-sin(theta + 3 * M_PI / 4), cos(theta + 3 * M_PI / 4), 1 / (2 * ROBOT_RADIUS)},
        {-sin(theta + 5 * M_PI / 4), cos(theta + 5 * M_PI / 4), 1 / (2 * ROBOT_RADIUS)},
        {-sin(theta + 7 * M_PI / 4), cos(theta + 7 * M_PI / 4), 1 / (2 * ROBOT_RADIUS)}
    };

    // Compute the robot's linear velocity (Vx, Vy) and angular velocity (omega)
    double Vx = 0, Vy = 0, omega = 0;
    for (int i = 0; i < 4; ++i)
    {
        Vx -= .01*J[i][0] * wheel_speeds[i];
        Vy += .01*J[i][1] * wheel_speeds[i];
        omega -= .01*J[i][2] * wheel_speeds[i];
    }

    // Update robot pose
    robot_pose.x += Vx * DT * cos(robot_pose.theta) - Vy * DT * sin(robot_pose.theta);
    robot_pose.y += Vx * DT * sin(robot_pose.theta) + Vy * DT * cos(robot_pose.theta);
    robot_pose.theta += omega * DT;

    // Normalize theta to [0, 2*pi]
    if (robot_pose.theta > TWO_PI)
        robot_pose.theta -= TWO_PI;
    if (robot_pose.theta < 0)
        robot_pose.theta += TWO_PI;

    // Publish the updated pose
    static ros::NodeHandle nh;
    static ros::Publisher pose_pub = nh.advertise<geometry_msgs::Pose2D>("robot_pose", 10);
    pose_pub.publish(robot_pose);
}

int main(int argc, char** argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "pose_estimation_node");
    ros::NodeHandle nh;

    // Initialize robot pose
    robot_pose.x = 0.0;
    robot_pose.y = 0.0;
    robot_pose.theta = 0.0;

    // Subscribers for individual encoder values
    ros::Subscriber encoder1_sub = nh.subscribe("motor1_encoder", 10, encoder1Callback);
    ros::Subscriber encoder2_sub = nh.subscribe("motor2_encoder", 10, encoder2Callback);
    ros::Subscriber encoder3_sub = nh.subscribe("motor3_encoder", 10, encoder3Callback);
    ros::Subscriber encoder4_sub = nh.subscribe("motor4_encoder", 10, encoder4Callback);

    // Set the update rate
    ros::Rate rate(50); // 100 Hz

    while (ros::ok())
    {
        if (!is_initialized) {
            sleep(5);
        }
        ros::spinOnce();
        calculatePose();
        rate.sleep();
    }

    return 0;
}
