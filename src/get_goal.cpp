#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "goal_plotter/goal.hpp"
#include "goal_plotter_messages/srv/sgoal.hpp"
#include "rclcpp/rclcpp.hpp"

class get_goal : public rclcpp::Node {
 public:
  get_goal() : Node("sub_goal_node") {
    selected_goal.x = 0;
    selected_goal.y = 0;
    selected_goal.z = 0;
    selected_goal.w = 0;

    auto goal_callback =
        [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          RCLCPP_INFO(
              rclcpp::get_logger("rclcpp"),
              "[get_goal] Received goal values: x: %f y: %f z: %f w: %f",
              msg->pose.position.x, msg->pose.position.y,
              msg->pose.orientation.z, msg->pose.orientation.w);
          selected_goal.x = msg->pose.position.x;
          selected_goal.y = msg->pose.position.y;
          selected_goal.z = msg->pose.orientation.z;
          selected_goal.w = msg->pose.orientation.w;
        };

    auto send_goal =
        [this](const std::shared_ptr<goal_plotter_messages::srv::Sgoal::Request> request,
               const std::shared_ptr<goal_plotter_messages::srv::Sgoal::Response>
                   response) {
          data = request->data;
          // send over the selected goal.
          response->x = selected_goal.x;
          response->y = selected_goal.y;
          response->z = selected_goal.z;
          response->w = selected_goal.w;

          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sent goal values");
        };

    goal_subscriber =
        this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, goal_callback);
    goal_service = this->create_service<goal_plotter_messages::srv::Sgoal>(
        "get_curr_goal", send_goal);
  }

 private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr
      goal_subscriber;
  rclcpp::Service<goal_plotter_messages::srv::Sgoal>::SharedPtr goal_service;
  goal_plotter::goal selected_goal;
  bool data = false;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<get_goal>());
  rclcpp::shutdown();
  return 0;
}
