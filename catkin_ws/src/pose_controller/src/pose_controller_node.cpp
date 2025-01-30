#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Vector3.h>
#include <cmath>

class PoseController {
public:
    PoseController() {
        ros::NodeHandle nh;

        // Load PID parameters from parameter server
        nh.param("kp_x", kp_x, 20.0);
        nh.param("ki_x", ki_x, 0.0);
        nh.param("kd_x", kd_x, 0.1);

        nh.param("kp_y", kp_y, 20.0);
        nh.param("ki_y", ki_y, 0.0);
        nh.param("kd_y", kd_y, 0.1);

        nh.param("kp_theta", kp_theta, 15.0);
        nh.param("ki_theta", ki_theta, 0.0);
        nh.param("kd_theta", kd_theta, 0.0);

        // Stopping thresholds (5% of goal)
        nh.param("stop_threshold_position", stop_threshold_position, 0.005);  // 5% of total distance
        nh.param("stop_threshold_theta", stop_threshold_theta, 0.005);  // 5% of total angle (radians)

        // Subscribers & Publisher
        desired_pose_sub = nh.subscribe("/desired_pose", 10, &PoseController::desiredPoseCallback, this);
        current_pose_sub = nh.subscribe("/robot_pose", 10, &PoseController::currentPoseCallback, this);
        local_velocity_pub = nh.advertise<geometry_msgs::Vector3>("/local_velocities", 10);

        last_time = ros::Time::now();
    }

    void spin() {
        ros::Rate rate(50);  // Run at 50Hz
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::Subscriber desired_pose_sub;
    ros::Subscriber current_pose_sub;
    ros::Publisher local_velocity_pub;

    geometry_msgs::Pose2D desired_pose;
    geometry_msgs::Pose2D current_pose;

    double kp_x, ki_x, kd_x;
    double kp_y, ki_y, kd_y;
    double kp_theta, ki_theta, kd_theta;

    double stop_threshold_position;
    double stop_threshold_theta;

    double prev_error_x = 0, prev_error_y = 0, prev_error_theta = 0;
    double integral_x = 0, integral_y = 0, integral_theta = 0;
    ros::Time last_time;
	double wrapAngle(double angle) {
		while (angle > M_PI) angle -= 2 * M_PI;
		while (angle < -M_PI) angle += 2 * M_PI;
		return angle;
	}
    void desiredPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        desired_pose = *msg;
    }

    void currentPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        current_pose = *msg;

        ros::Time now = ros::Time::now();
        double dt = (now - last_time).toSec();
        last_time = now;

        if (dt <= 0) return;  // Prevent division by zero

        // Compute errors
        double error_x = desired_pose.x - current_pose.x;
        double error_y = desired_pose.y - current_pose.y;
        double error_theta = wrapAngle(desired_pose.theta - current_pose.theta);

        // Compute Euclidean distance error
        double position_error = sqrt(error_x * error_x + error_y * error_y);
        double theta_error = fabs(error_theta);

        // Check stopping condition
        if (position_error < stop_threshold_position && theta_error < stop_threshold_theta) {
            geometry_msgs::Vector3 stop_velocity;
            stop_velocity.x = 0;
            stop_velocity.y = 0;
            stop_velocity.z = 0;
            local_velocity_pub.publish(stop_velocity);
            return;
        }

        // Compute PID terms
        integral_x += error_x * dt;
        integral_y += error_y * dt;
        integral_theta += error_theta * dt;

        double derivative_x = (error_x - prev_error_x) / dt;
        double derivative_y = (error_y - prev_error_y) / dt;
        double derivative_theta = (error_theta - prev_error_theta) / dt;

        // Compute control outputs
        geometry_msgs::Vector3 local_velocity;
        local_velocity.x = kp_x * error_x + ki_x * integral_x + kd_x * derivative_x;
        local_velocity.y = kp_y * error_y + ki_y * integral_y + kd_y * derivative_y;
        local_velocity.z = -1*(kp_theta * error_theta + ki_theta * integral_theta + kd_theta * derivative_theta);

        // Publish local velocity
        local_velocity_pub.publish(local_velocity);

        // Store previous errors
        prev_error_x = error_x;
        prev_error_y = error_y;
        prev_error_theta = error_theta;
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pose_controller");

    PoseController controller;
    controller.spin();  // Run the control loop at 50Hz

    return 0;
}
