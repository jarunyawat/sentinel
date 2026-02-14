#include <sentinel_behavior/create_circle.hpp>

CreateCircle::CreateCircle(const std::string& node_name): Node(node_name){
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  this->action_server_ = rclcpp_action::create_server<GenCircle>(
      this,
      "compute_circle_path",
      std::bind(&CreateCircle::handle_goal, this, _1, _2),
      std::bind(&CreateCircle::handle_cancel, this, _1),
      std::bind(&CreateCircle::handle_accepted, this, _1));

    path_publisher_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
}

rclcpp_action::GoalResponse CreateCircle::handle_goal(
const rclcpp_action::GoalUUID & uuid,
std::shared_ptr<const GenCircle::Goal> goal)
{
    RCLCPP_INFO(this->get_logger(), "Received goal request");
    (void)uuid;
    (void)goal;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CreateCircle::handle_cancel(
const std::shared_ptr<GoalHandleGenCircle> goal_handle)
{
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void CreateCircle::handle_accepted(const std::shared_ptr<GoalHandleGenCircle> goal_handle)
{
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&CreateCircle::execute, this, _1), goal_handle}.detach();
}

void CreateCircle::execute(const std::shared_ptr<GoalHandleGenCircle> goal_handle){
    const auto goal = goal_handle->get_goal();
    auto path_msg = genCirclePath(goal->radius, goal->theta, goal->direction);
    path_publisher_->publish(path_msg);
    auto result = std::make_shared<GenCircle::Result>();
    result->path = path_msg;
    goal_handle->succeed(result);
    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
}

nav_msgs::msg::Path CreateCircle::genCirclePath(double radius, double theta, bool dir){
  nav_msgs::msg::Path path_in_base;
  path_in_base.header.frame_id = "base_link";
  path_in_base.header.stamp = this->now();

  float sum_theta = 0.0;

  while(abs(sum_theta) < theta) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header = path_in_base.header;
    pose.pose.position.x = radius * (1 - cos(sum_theta));
    pose.pose.position.y =  -radius * sin(sum_theta);
    tf2::Quaternion q;
    q.setRPY(0, 0, sum_theta);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();
    path_in_base.poses.push_back(pose);
    if(dir){
        sum_theta += 0.1;
    }
    else{
        sum_theta -= 0.1;
    }
  }
  nav_msgs::msg::Path path_in_map = transformPathToMap(path_in_base);
  path_ = path_in_map;
  return path_in_map;
}

// Transform single pose from base_link to map
bool CreateCircle::transformPoseToMap(const geometry_msgs::msg::PoseStamped& input_pose,
                        geometry_msgs::msg::PoseStamped& output_pose)
{
    try
    {
        // Wait for transform to be available (with timeout)
        if (!tf_buffer_->canTransform("map", input_pose.header.frame_id, 
                                    input_pose.header.stamp, 
                                    rclcpp::Duration::from_seconds(1.0)))
        {
            RCLCPP_WARN(this->get_logger(), 
                "Transform from %s to map not available", 
                input_pose.header.frame_id.c_str());
            return false;
        }
        
        // Perform the transformation
        tf_buffer_->transform(input_pose, output_pose, "map");
        
        return true;
    }
    catch (tf2::TransformException& ex)
    {
        RCLCPP_ERROR(this->get_logger(), 
            "Transform failed: %s", ex.what());
        return false;
    }
}

nav_msgs::msg::Path CreateCircle::transformPathToMap(const nav_msgs::msg::Path& input_path)
{
    nav_msgs::msg::Path transformed_path;
    transformed_path.header.frame_id = "map";
    transformed_path.header.stamp = this->get_clock()->now();
    
    // Get the latest transform timestamp
    rclcpp::Time latest_time = rclcpp::Time(0); // Latest available
    
    transformed_path.poses.reserve(input_path.poses.size());
    
    for (const auto& pose_stamped : input_path.poses)
    {
        try
        {
            geometry_msgs::msg::PoseStamped temp_pose = pose_stamped;
            temp_pose.header.stamp = latest_time; // Use latest transform
            
            geometry_msgs::msg::PoseStamped transformed_pose;
            if (transformPoseToMap(temp_pose, transformed_pose))
            {
                // Keep original timestamp in transformed pose
                transformed_pose.header.stamp = pose_stamped.header.stamp;
                transformed_path.poses.push_back(transformed_pose);
            }
        }
        catch (const std::exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), 
                "Exception during latest transform: %s", e.what());
        }
    }
    
    return transformed_path;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CreateCircle>("create_circle_server"));
  rclcpp::shutdown();
  return 0;
}