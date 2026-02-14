#ifndef FOLLOW_CIRCLE_HPP
#define FOLLOW_CIRCLE_HPP

#include<rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sentinel_interfaces/action/nav_circle.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <cmath>
#include <Eigen/Dense>

using namespace std::placeholders;
using namespace sentinel_interfaces::action;
using NavCircle = sentinel_interfaces::action::NavCircle;
using GoalHandleNavCircle = rclcpp_action::ServerGoalHandle<NavCircle>;

class FollowCircle : public rclcpp::Node {
    public:
        FollowCircle(const std::string& node_name);
    private:
        nav_msgs::msg::Path path_recieve;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

        rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
        rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
        rclcpp_action::Server<NavCircle>::SharedPtr follow_path_server_;
        rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavCircle::Goal> goal);
        rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavCircle> goal_handle);
        void handle_accepted(const std::shared_ptr<GoalHandleNavCircle> goal_handle);
        void execute(const std::shared_ptr<GoalHandleNavCircle> goal_handle);

        int truncateNearest(double robot_x, double robot_y);

        void path_cb(const nav_msgs::msg::Path::SharedPtr msg);
};

#endif