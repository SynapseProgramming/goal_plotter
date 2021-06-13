#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "goal_plotter/goal.h"
#include "goal_plotter/srv/sgoal.hpp"
#include "rclcpp/rclcpp.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

using namespace std::chrono_literals;

class plot : public rclcpp::Node {
 public:
  plot() : Node("goal_plotter") {
    sub_goal_client =
        this->create_client<goal_plotter::srv::Sgoal>("get_curr_goal");
  }

  void main_menu() {
    std::string input;
    std::cout << "(ag) Add a new goal\n";
    std::cout << "(ls) List all goals\n";
    std::cout << "(rm) Remove a specific goal\n";
    std::cout << "(ck) Check if a goal exists\n";
    std::cout << "(sv) Save and export goal file\n";

    std::cin >> input;
    if (input == "ag") {
      new_goal();
    } else if (input == "ls")
      list_goals();
    else {
      std::cout << "Please re-enter one of the aforementioned acronyms.\n";
      main_menu();
    }
  }

  // this function returns a shared_ptr which points to the current instant.
  std::shared_ptr<plot> shared_plot_from_this() {
    return std::static_pointer_cast<plot>((shared_from_this()));
  }
  // list_goals would print out all of the stored goals in goal_map
  void list_goals() {
    if (goal_map.size() == 0)
      std::cout << "No goals available.\n";
    else {
      for (auto it = goal_map.begin(); it != goal_map.end(); it++) {
        std::cout << it->first << " x:" << it->second.x << " y:" << it->second.y
                  << " z:" << it->second.z << " w:" << it->second.w << "\n";
      }
    }
    main_menu();
  }

  // This function would add a new goal into the map. With a custom goal_name
  void new_goal() {
    std::cout << "Please enter a name for your goal.\n";
    std::string goal_name;
    std::cin >> goal_name;
    std::cout << "To confirm goal:\n";
    if (wait_confirmation()) {
      std::shared_ptr<plot> current_node_ptr = shared_plot_from_this();
      auto request = std::make_shared<goal_plotter::srv::Sgoal::Request>();
      request->data = false;
      // wait for the server to be up
      while (!sub_goal_client->wait_for_service(1s)) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                       "Interrupted while waiting for the service. Exiting.");
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_goal, waiting again...");
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "get_goal has started");

      auto result = sub_goal_client->async_send_request(request);
      // Wait for the result.
      if (rclcpp::spin_until_future_complete(current_node_ptr, result) ==
          rclcpp::FutureReturnCode::SUCCESS) {
        RCLCPP_INFO(
            rclcpp::get_logger("rclcpp"),
            "[goal_plotter] Received goal values: x: %f y: %f z: %f w: %f",
            result.get()->x, result.get()->y, result.get()->z, result.get()->w);
        // copy over the selected goal positions
        selected_goal.x = result.get()->x;
        selected_goal.y = result.get()->y;
        selected_goal.z = result.get()->z;
        selected_goal.w = result.get()->w;
        goal_map[goal_name] = selected_goal;
        main_menu();

      } else {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Failed to call service Goal not added to goal_map.");
        main_menu();
      }

    } else
      main_menu();
  }
  // the wait_confirmation function would prompt the user to enter (y\n).
  // Function returns true if (y) and false if (anything else if entered)
  bool wait_confirmation() {
    std::cout
        << "Please enter (y) for yes. Press and enter anything else for no.\n";
    std::string input;
    std::cin >> input;
    return input == "y" ? true : false;
  }

 private:
  std::map<std::string, goal_plotter::goal> goal_map;
  rclcpp::Client<goal_plotter::srv::Sgoal>::SharedPtr sub_goal_client;
  goal_plotter::goal selected_goal;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  std::shared_ptr<plot> node_ptr = std::make_shared<plot>();
  node_ptr->main_menu();
  rclcpp::spin(node_ptr);
  rclcpp::shutdown();
  return 0;
}
