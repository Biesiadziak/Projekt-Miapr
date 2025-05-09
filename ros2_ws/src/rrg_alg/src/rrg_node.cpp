#include <cmath>
#include <string>
#include <memory>

#include "nav2_util/node_utils.hpp"

#include "rrg_alg/rrg_node.hpp"

namespace nav2_straightline_planner
{

void StraightLine::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();

  // Parameter initialization
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".interpolation_resolution", rclcpp::ParameterValue(
      0.1));
  node_->get_parameter(name_ + ".interpolation_resolution", interpolation_resolution_);

  // Configure everithing
  gen_ = std::mt19937(rd());
  marker_pub_ = this->node_->create_publisher<visualization_msgs::msg::Marker>("random_point_marker", 10);
  id = 0;
}

void StraightLine::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void StraightLine::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

std::tuple<float,float> StraightLine::randomPoint()
{
  double width = costmap_->getSizeInMetersX();
  double height = costmap_->getSizeInMetersY();

  std::uniform_real_distribution<> distrib(0.0, 1.0);

  float x = (distrib(gen_) * width) / 3 - width/6;
  float y = (distrib(gen_) * height) / 3 - height/6;

  return std::make_tuple(x, y);    
}

void StraightLine::publishMarker(std::tuple<float, float> point)
{
  auto [x, y] = point;
  visualization_msgs::msg::Marker marker;

  marker.header.frame_id = "map"; // Choose the correct frame
  marker.header.stamp = node_->get_clock()->now();
  marker.ns = "random_point %f", id;
  marker.id = id;
  id++;
  marker.type = visualization_msgs::msg::Marker::SPHERE;
  marker.action = visualization_msgs::msg::Marker::ADD;

  // Set the position of the marker
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0; // Keep it at the ground level
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // Set the scale of the marker (size of the sphere)
  marker.scale.x = 0.5; // Diameter of the sphere
  marker.scale.y = 0.5;
  marker.scale.z = 0.5;

  // Set the color (RGBA)
  marker.color.a = 1.0; // Alpha (transparency)
  marker.color.r = 1.0; // Red
  marker.color.g = 0.0; // Green
  marker.color.b = 0.0; // Blue

  // Publish the marker
  marker_pub_->publish(marker);
  RCLCPP_INFO(this->node_->get_logger(), "Marker published");
}

nav_msgs::msg::Path StraightLine::createPlan(const geometry_msgs::msg::PoseStamped & start, const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> /*cancel_checker*/)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;
  // calculating the number of loops for current value of interpolation_resolution_
  int total_number_of_loop = std::hypot(
    goal.pose.position.x - start.pose.position.x,
    goal.pose.position.y - start.pose.position.y) /
    interpolation_resolution_;
  double x_increment = (goal.pose.position.x - start.pose.position.x) / total_number_of_loop;
  double y_increment = (goal.pose.position.y - start.pose.position.y) / total_number_of_loop;

  for (int i = 0; i < total_number_of_loop; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start.pose.position.x + x_increment * i;
    pose.pose.position.y = start.pose.position.y + y_increment * i;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }
  for (int i=0; i<10; i++)
  {
    std::tuple<float, float> point = this->randomPoint();
    RCLCPP_INFO(this->node_->get_logger(), "Random point: %f, %f", std::get<0>(point), std::get<1>(point));
    publishMarker(point);
  }

  geometry_msgs::msg::PoseStamped goal_pose = goal;
  goal_pose.header.stamp = node_->now();
  goal_pose.header.frame_id = global_frame_;
  global_path.poses.push_back(goal_pose);

  return global_path;
}
} // namespace nav2_straightline_planner

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_straightline_planner::StraightLine, nav2_core::GlobalPlanner)