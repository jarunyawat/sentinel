#include<sentinel_behavior/follow_circle.hpp>

FollowCircle::FollowCircle(const std::string& node_name) : Node(node_name){
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
        "plan",
        10,
        std::bind(
            &FollowCircle::path_cb,
            this,
            _1
        )
    );
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    follow_path_server_ = rclcpp_action::create_server<NavCircle>(
        this, "follow_circle",
        std::bind(&FollowCircle::handle_goal, this, _1, _2),
        std::bind(&FollowCircle::handle_cancel, this, _1),
        std::bind(&FollowCircle::handle_accepted, this, _1)
    );
}

void FollowCircle::path_cb(const nav_msgs::msg::Path::SharedPtr msg){
    path_recieve = *msg;
}

rclcpp_action::GoalResponse FollowCircle::handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const NavCircle::Goal> goal){
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse FollowCircle::handle_cancel(const std::shared_ptr<GoalHandleNavCircle> goal_handle){
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void FollowCircle::handle_accepted(const std::shared_ptr<GoalHandleNavCircle> goal_handle){
    std::thread{std::bind(&FollowCircle::execute, this, _1), goal_handle}.detach();
}

void FollowCircle::execute(const std::shared_ptr<GoalHandleNavCircle> goal_handle){
    rclcpp::Rate loop_rate(10.0);
    Eigen::Matrix2d R;
    while(true){
        if (goal_handle->is_canceling()) {
            RCLCPP_WARN(this->get_logger(), "Goal canceled, stopping execute loop.");
            auto result = std::make_shared<sentinel_interfaces::action::NavCircle::Result>();
            goal_handle->canceled(result);
            return;  // Exit the execute() function
        }
        geometry_msgs::msg::TransformStamped t;
        double robot_x = 0.0;
        double robot_y = 0.0;
        tf2::Quaternion robot_yaw;
        try {
            t = tf_buffer_->lookupTransform(
                "map", "base_link",
                tf2::TimePointZero);
            // Extract quaternion
            robot_yaw.setX(t.transform.rotation.x);
            robot_yaw.setY(t.transform.rotation.y);
            robot_yaw.setZ(t.transform.rotation.z);
            robot_yaw.setW(t.transform.rotation.w);
            
            double roll, pitch, yaw;
            tf2::Matrix3x3(robot_yaw).getRPY(roll, pitch, yaw);
            
            R << std::cos(yaw), -std::sin(yaw),
                 std::sin(yaw),  std::cos(yaw);
            
            robot_x = t.transform.translation.x;
            robot_y = t.transform.translation.y;
        } catch (const tf2::TransformException & ex) {
          RCLCPP_INFO(this->get_logger(), "Could not transform map to base_link: %s", ex.what());
          return;
        }
        // RCLCPP_INFO(this->get_logger(), "x: %.3f y: %.3f", robot_x, robot_y);

        if(truncateNearest(robot_x, robot_y) == 1){
            double x_path = path_recieve.poses[0].pose.position.x;
            double y_path = path_recieve.poses[0].pose.position.y;
            tf2::Quaternion path_q(
                path_recieve.poses[0].pose.orientation.x,
                path_recieve.poses[0].pose.orientation.y,
                path_recieve.poses[0].pose.orientation.z,
                path_recieve.poses[0].pose.orientation.w);

            double delta_x = x_path - robot_x;
            double delta_y = y_path - robot_y;
            tf2::Quaternion q_relative = path_q * robot_yaw.inverse();
            q_relative.normalize();
            double delta_roll, delta_pitch, delta_yaw;
            tf2::Matrix3x3(q_relative).getRPY(delta_roll, delta_pitch, delta_yaw);
            
            double delta_vec = (std::hypot(delta_x, delta_y));

            double cmd_vel = delta_vec * 3.0;
            double normalize_delta_x = delta_x / delta_vec;
            double normalize_delta_y = delta_y / delta_vec;
            if(cmd_vel > 0.3){
                cmd_vel = 0.3;
            }

            double vel_x_body = cmd_vel * normalize_delta_x;
            double vel_y_body = cmd_vel * normalize_delta_y;
            double vel_z_body = delta_yaw * 3.0;;
            if(vel_z_body > 0.3){
                vel_z_body = 0.3;
            }
            else if(vel_z_body < -0.3){
                vel_z_body = -0.3;
            }

            Eigen::Vector2d v_map(vel_x_body, vel_y_body);
            Eigen::Vector2d v_base = R.transpose() * v_map;
            // RCLCPP_INFO(this->get_logger(), "vel_x_body: %.3f vel_y_body: %.3f", vel_x_body, vel_y_body);

            auto cmd_vel_msg = geometry_msgs::msg::Twist();
            cmd_vel_msg.linear.x = v_base[0];
            cmd_vel_msg.linear.y = v_base[1];
            cmd_vel_msg.angular.z = vel_z_body;
            cmd_vel_pub_->publish(cmd_vel_msg);
            double goal_x = path_recieve.poses.back().pose.position.x;
            double goal_y = path_recieve.poses.back().pose.position.y;
            if(std::hypot(goal_x - robot_x, goal_y - robot_y) < 0.05){
                RCLCPP_WARN(this->get_logger(), "Goal reached, stopping execute loop.");
                cmd_vel_msg.linear.x = 0.0;
                cmd_vel_msg.linear.y = 0.0;
                cmd_vel_msg.angular.z = 0.0;
                cmd_vel_pub_->publish(cmd_vel_msg);
                auto result = std::make_shared<sentinel_interfaces::action::NavCircle::Result>();
                goal_handle->succeed(result);
                return;  // Exit the execute() function
            }
        }
        loop_rate.sleep();
    }
}

int FollowCircle::truncateNearest(double robot_x, double robot_y){
    // Make sure we have a path to work with
    if (path_recieve.poses.empty()) {
        RCLCPP_WARN(this->get_logger(), "path empty");
        return -1;
    }

    // Find the first waypoint farther than 0.1 m from the robot
    for (size_t i = 0; i < path_recieve.poses.size(); ++i) {
        const auto &waypoint = path_recieve.poses[i];
        double x = waypoint.pose.position.x;
        double y = waypoint.pose.position.y;
        double dist = std::hypot(x - robot_x, y - robot_y);

        if (dist > 0.1) {
            // Erase all poses before this index (truncate the path)
            path_recieve.poses.erase(path_recieve.poses.begin(), path_recieve.poses.begin() + i);
            return 1;  // success
        }
    }

    return 1; // no suitable point found
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<FollowCircle>("follow_circle_server"));
  rclcpp::shutdown();
  return 0;
}