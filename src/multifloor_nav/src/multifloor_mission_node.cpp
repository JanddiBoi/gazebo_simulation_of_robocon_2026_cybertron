#include <chrono>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <nav2_msgs/srv/load_map.hpp>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <yaml-cpp/yaml.h>

#include "multifloor_nav/transition_backends.hpp"

using namespace std::chrono_literals;
using NavigateToPose = nav2_msgs::action::NavigateToPose;

class MultiFloorMission : public rclcpp::Node
{
public:
  MultiFloorMission(const std::string & config_path, std::shared_ptr<TransitionBackend> backend)
  : Node("multifloor_mission"), backend_(std::move(backend)), current_floor_(1)
  {
    cfg_ = YAML::LoadFile(config_path);

    load_map_client_ = create_client<nav2_msgs::srv::LoadMap>("/map_server/load_map");
    clear_global_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/global_costmap/clear_entirely_global_costmap");
    clear_local_client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>(
      "/local_costmap/clear_entirely_local_costmap");
    init_pose_pub_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
      "/initialpose", 10);
    nav_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
  }

  void changeFloor(const std::string & transition_id, int target_floor)
  {
    auto t = cfg_["transitions"][transition_id];
    auto target_node = t[std::to_string(target_floor)];

    TransitionPose pose;
    pose.x = target_node["x"].as<double>();
    pose.y = target_node["y"].as<double>();
    pose.yaw = target_node["yaw"].as<double>();
    pose.fx = target_node["floor_pose"]["x"].as<double>();
    pose.fy = target_node["floor_pose"]["y"].as<double>();
    pose.fz = target_node["floor_pose"]["z"].as<double>();

    RCLCPP_INFO(get_logger(), "Transitioning to floor %d...", target_floor);
    backend_->execute(transition_id, current_floor_, target_floor, pose);

    const std::string map_path =
      cfg_["floors"][std::to_string(target_floor)]["map"].as<std::string>();
    swapMap(map_path);
    publishInitialPose(pose.x, pose.y, pose.yaw);
    clearCostmaps();

    // simple settle delay; poll /amcl_pose covariance for something stricter
    std::this_thread::sleep_for(5s);

    current_floor_ = target_floor;
    RCLCPP_INFO(get_logger(), "Now on floor %d", target_floor);
  }

  bool goTo(double x, double y, double yaw = 0.0)
  {
    if (!nav_client_->wait_for_action_server(5s)) {
      RCLCPP_ERROR(get_logger(), "navigate_to_pose action server not available");
      return false;
    }

    NavigateToPose::Goal goal;
    goal.pose.header.frame_id = "map";
    goal.pose.header.stamp = now();
    goal.pose.pose.position.x = x;
    goal.pose.pose.position.y = y;
    goal.pose.pose.orientation.z = std::sin(yaw / 2.0);
    goal.pose.pose.orientation.w = std::cos(yaw / 2.0);

    auto future_goal_handle = nav_client_->async_send_goal(goal);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future_goal_handle) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    auto goal_handle = future_goal_handle.get();
    if (!goal_handle) {return false;}

    auto result_future = nav_client_->async_get_result(goal_handle);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), result_future) !=
      rclcpp::FutureReturnCode::SUCCESS)
    {
      return false;
    }

    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

private:
  void swapMap(const std::string & map_path)
  {
    load_map_client_->wait_for_service();
    auto req = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    req->map_url = map_path;
    auto future = load_map_client_->async_send_request(req);
    rclcpp::spin_until_future_complete(get_node_base_interface(), future);
  }

  void publishInitialPose(double x, double y, double yaw)
  {
    geometry_msgs::msg::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "map";
    msg.header.stamp = now();
    msg.pose.pose.position.x = x;
    msg.pose.pose.position.y = y;
    msg.pose.pose.orientation.z = std::sin(yaw / 2.0);
    msg.pose.pose.orientation.w = std::cos(yaw / 2.0);
    // tight-ish covariance since we KNOW where the robot is (sim); loosen
    // this on real hardware if the placement is less precise
    msg.pose.covariance[0] = 0.05;
    msg.pose.covariance[7] = 0.05;
    msg.pose.covariance[35] = 0.05;
    init_pose_pub_->publish(msg);
  }

  void clearCostmaps()
  {
    for (auto & client : {clear_global_client_, clear_local_client_}) {
      client->wait_for_service();
      auto req = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
      auto future = client->async_send_request(req);
      rclcpp::spin_until_future_complete(get_node_base_interface(), future);
    }
  }

  YAML::Node cfg_;
  std::shared_ptr<TransitionBackend> backend_;
  int current_floor_;

  rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr load_map_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_global_client_;
  rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr clear_local_client_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_pub_;
  rclcpp_action::Client<NavigateToPose>::SharedPtr nav_client_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto backend = std::make_shared<SimTeleportBackend>("multifloor", "waffle");
  auto mission = std::make_shared<MultiFloorMission>("config/floors.yaml", backend);

  mission->goTo(3.5, 0.8);                    // drive to lift on floor 1
  mission->changeFloor("lift_A", 2);
  mission->goTo(1.0, 2.0);                    // goal on floor 2
  mission->changeFloor("lift_A", 3);
  mission->goTo(0.5, 0.5);                    // goal on floor 3

  rclcpp::shutdown();
  return 0;
}