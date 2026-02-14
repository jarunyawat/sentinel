#ifndef CREATE_CIRCLE_HPP_
#define CREATE_CIRCLE_HPP_
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <memory>
#include "tf2/LinearMath/Matrix3x3.h"
#include <chrono>
#include <rclcpp_action/rclcpp_action.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sentinel_interfaces/action/gen_circle.hpp>

using namespace std::chrono_literals;
using namespace std::placeholders;
using GenCircle = sentinel_interfaces::action::GenCircle;
using GoalHandleGenCircle = rclcpp_action::ServerGoalHandle<GenCircle>;

class CreateCircle : public rclcpp::Node {
    public:
        CreateCircle(const std::string& node_name);
    private:
        nav_msgs::msg::Path path_;

        rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_publisher_;
        rclcpp_action::Server<GenCircle>::SharedPtr action_server_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GenCircle::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGenCircle> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleGenCircle> goal_handle);
        void execute(const std::shared_ptr<GoalHandleGenCircle> goal_handle);

        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        nav_msgs::msg::Path genCirclePath(double radius, double theta, bool dir);
        bool transformPoseToMap(const geometry_msgs::msg::PoseStamped& input_pose, geometry_msgs::msg::PoseStamped& output_pose);
        nav_msgs::msg::Path transformPathToMap(const nav_msgs::msg::Path& input_path);
};
#endif