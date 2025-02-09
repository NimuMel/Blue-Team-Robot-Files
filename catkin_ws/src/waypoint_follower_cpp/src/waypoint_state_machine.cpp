#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Bool.h>
#include <vector>

class WaypointStateMachine {
public:
    WaypointStateMachine(ros::NodeHandle& nh) : nh_(nh), current_index_(0) {
        // Define your list of waypoints (x, y, theta)
        geometry_msgs::Pose2D wp1;
        wp1.x = 0.0;
        wp1.y = 0.0;
        wp1.theta = 0.0;

        geometry_msgs::Pose2D wp2;
        wp2.x = 0.0;
        wp2.y = -0.5;
        wp2.theta = 0.0; // 45 degrees in radians

        geometry_msgs::Pose2D wp3;
        wp3.x = 0.0;
        wp3.y = -0.5;
        wp3.theta = -1.571; // 90 degrees in radians

        waypoints_.push_back(wp1);
        waypoints_.push_back(wp2);
        waypoints_.push_back(wp3);

        // Publisher to send waypoints
        waypoint_pub_ = nh_.advertise<geometry_msgs::Pose2D>("/waypoint", 10);

        // Subscriber to receive goal_reached messages
        goal_reached_sub_ = nh_.subscribe("/goal_reached", 10, &WaypointStateMachine::goalReachedCallback, this);

        ROS_INFO("Starting waypoint state machine.");
        publishCurrentWaypoint();
    }

    void publishCurrentWaypoint() {
        if (current_index_ < waypoints_.size()) {
            geometry_msgs::Pose2D waypoint = waypoints_[current_index_];
            ROS_INFO("Publishing waypoint %zu: x=%.2f, y=%.2f, theta=%.2f",
                     current_index_ + 1, waypoint.x, waypoint.y, waypoint.theta);
            waypoint_pub_.publish(waypoint);
        } else {
            ROS_INFO("All waypoints have been published and reached.");
            ros::shutdown();
        }
    }

    void goalReachedCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (msg->data) {
            ROS_INFO("Goal reached for waypoint %zu.", current_index_ + 1);
            current_index_++;
            publishCurrentWaypoint();
        } else {
            //ROS_WARN("Received goal_reached signal with False value.");
        }
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher waypoint_pub_;
    ros::Subscriber goal_reached_sub_;
    std::vector<geometry_msgs::Pose2D> waypoints_;
    size_t current_index_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "waypoint_state_machine");
    ros::NodeHandle nh;

    WaypointStateMachine wsm(nh);

    ros::spin();
    return 0;
}
