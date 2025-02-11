#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <cmath> // For sqrt, pow, fabs, and M_PI

class WaypointStateMachine {
public:
    WaypointStateMachine(ros::NodeHandle& nh)
        : nh_(nh),
          current_index_(0),
          position_tolerance_(0.005),     // Position tolerance in meters
          orientation_tolerance_(0.005) { // Orientation tolerance in radians
        // Initialize waypoints
        initializeWaypoints();

        // Publisher to send waypoints
        waypoint_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/waypoint", 10);
		scoop_pub = nh.advertise<std_msgs::Bool>("/OpenCR/scoop", 1000);
        // Subscriber to receive robot's current pose
        robot_pose_sub_ = nh_.subscribe("/robot_pose", 10, &WaypointStateMachine::robotPoseCallback, this);

        ROS_INFO("Starting waypoint state machine.");
        publishCurrentWaypoint();
    }

    void initializeWaypoints() {
        // Define your list of waypoints (x, y, theta)
        geometry_msgs::Pose2D wp1;
        wp1.x = 0.0;
        wp1.y = 0.0;
        wp1.theta = 0.0;

        geometry_msgs::Pose2D wp2;
        wp2.x = 0.0;
        wp2.y = 0.4;
        wp2.theta = 0.0;

        geometry_msgs::Pose2D wp3;
        wp3.x = 0.0;
        wp3.y = 0.45;
        wp3.theta = 0.0; // -90 degrees in radians
		
		
		geometry_msgs::Pose2D wp4;
        wp4.x = 0.0;
        wp4.y = 0.45;
        wp4.theta = -1.571;
		
		geometry_msgs::Pose2D wp5;
        wp5.x = 0.4;
        wp5.y = 0.45;
        wp5.theta = -1.571;
		
		geometry_msgs::Pose2D wp6;
        wp6.x = 1.0;
        wp6.y = 0.45;
        wp6.theta = -1.571;
		
		geometry_msgs::Pose2D wp7;
        wp7.x = 1.0;
        wp7.y = 0.45;
        wp7.theta = 1.571;
		
		geometry_msgs::Pose2D wp8;
        wp8.x = 0.7;
        wp8.y = 0.45;
        wp8.theta = 1.571;
		
		geometry_msgs::Pose2D wp9;
        wp9.x = 0.0;
        wp9.y = 0.45;
        wp9.theta = 1.571;

        waypoints_.push_back(wp1);
        waypoints_.push_back(wp2);
        waypoints_.push_back(wp3);
		waypoints_.push_back(wp4);
		waypoints_.push_back(wp5);
		// waypoints_.push_back(wp6);
		// waypoints_.push_back(wp7);
		// waypoints_.push_back(wp8);
		// waypoints_.push_back(wp9);
    }

    void publishWaypoint(const geometry_msgs::Pose2D& waypoint) {
        ROS_INFO("Publishing waypoint: x=%.2f, y=%.2f, theta=%.2f",
                 waypoint.x, waypoint.y, waypoint.theta);
        waypoint_pub_.publish(waypoint);
    }

    void publishCurrentWaypoint() {
        if (current_index_ < waypoints_.size()) {
            current_waypoint_ = waypoints_[current_index_];
			if(current_index_ == 2){
				std_msgs::Bool pub;
				pub.data = true;
				scoop_pub.publish(pub);
			}
			else{
				std_msgs::Bool pub;
				pub.data = false;
				scoop_pub.publish(pub);
			}
            publishWaypoint(current_waypoint_);
        } else {
            ROS_INFO("All waypoints have been published and reached.");
            ros::shutdown();
        }
    }

    void robotPoseCallback(const geometry_msgs::Pose2D::ConstPtr& msg) {
        double position_error = calculatePositionError(*msg, current_waypoint_);
        double orientation_error = calculateOrientationError(*msg, current_waypoint_);

        if (position_error <= position_tolerance_ && orientation_error <= orientation_tolerance_) {
            ROS_INFO("Goal reached for waypoint %zu.", current_index_ + 1);
            current_index_++;
            publishCurrentWaypoint();
        } else {
            // Uncomment for debugging
            // ROS_INFO("Position error: %.3f, Orientation error: %.3f", position_error, orientation_error);
        }
    }

    double calculatePositionError(const geometry_msgs::Pose2D& pose1, const geometry_msgs::Pose2D& pose2) {
        return sqrt(pow(pose1.x - pose2.x, 2) + pow(pose1.y - pose2.y, 2));
    }

    double calculateOrientationError(const geometry_msgs::Pose2D& pose1, const geometry_msgs::Pose2D& pose2) {
        double error = pose1.theta - pose2.theta;
        // Normalize the angle to the range [-pi, pi]
        while (error > M_PI) error -= 2 * M_PI;
        while (error < -M_PI) error += 2 * M_PI;
        return fabs(error);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher waypoint_pub_;
	ros::Publisher scoop_pub;
    ros::Subscriber robot_pose_sub_;
    std::vector<geometry_msgs::Pose2D> waypoints_;
    geometry_msgs::Pose2D current_waypoint_;
    size_t current_index_;
    double position_tolerance_;
    double orientation_tolerance_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_state_machine");
    ros::NodeHandle nh;

    WaypointStateMachine wsm(nh);

    ros::spin();
    return 0;
}
