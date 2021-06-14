#include <chrono>
#include <iostream>
#include <map>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "goal_plotter/goal.hpp"
#include "goal_plotter/srv/sgoal.hpp"
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

using namespace std::chrono_literals;

/**
 * A small convenience function for converting a thread ID to a string
 **/
std::string string_thread_id() {
  auto hashed = std::hash<std::thread::id>()(std::this_thread::get_id());
  return std::to_string(hashed);
}

class plot : public rclcpp::Node {
 public:
  plot() : Node("goal_plotter") {
    sub_goal_client =
        this->create_client<goal_plotter::srv::Sgoal>("get_curr_goal");

    marker_array_pub =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/selected_goals", 10);
  }

  void main_menu() {
    std::string input;
    std::cout << "(ag) Add a new goal\n";
    std::cout << "(ls) List all goals\n";
    std::cout << "(rm) Remove a specific goal\n";
    std::cout << "(ck) Check if a goal exists\n";
    std::cout << "(sv) Save and export goal file\n";
    std::cout << "(md) Modify an existing goal\n";

    std::cin >> input;
    if (input == "ag") {
      new_goal();
    } else if (input == "ls") {
      list_goals();
    } else if (input == "rm") {
      remove_goal();
    } else if (input == "md") {
      modify_goal();
    } else {
      std::cout << "Please re-enter one of the aforementioned acronyms.\n";
      main_menu();
    }
  }

  // this function returns a shared_ptr which points to the current instant.
  std::shared_ptr<plot> shared_plot_from_this() {
    return std::static_pointer_cast<plot>((shared_from_this()));
  }
  // this function allows the user to modify and exising goal
  void modify_goal() {
    std::cout << "Enter name of goal to modify.\n";
    std::string goal_name;
    std::cin >> goal_name;
    if (goal_map.count(goal_name)) {
      std::cout << goal_name
                << " found. Please select a new pose.\n To confirm: ";
      if (wait_confirmation()) {
        update_selected_goal();
        goal_map[goal_name] = selected_goal;
        add_single_marker(visualization_msgs::msg::Marker::ADD, selected_goal,
                          marker_map[goal_name], goal_name);
      }
    } else {
      std::cout << "Goal not found.\n";
    }
    main_menu();
  }

  // this function would remove a specific goal from goal_map
  void remove_goal() {
    std::cout << "Enter name of goal to remove.\n";
    std::string goal_name;
    std::cin >> goal_name;
    if (goal_map.count(goal_name)) {
      std::cout << goal_name << " found. Do you wish to remove this goal?\n";
      if (wait_confirmation()) {
        goal_map.erase(goal_name);
        add_single_marker(visualization_msgs::msg::Marker::DELETE,
                          selected_goal, marker_map[goal_name], goal_name);
        marker_map.erase(goal_name);
        std::cout << "successfully erased: " << goal_name << "\n";
      }
    } else
      std::cout << "Goal not found.\n";

    main_menu();
  }
  // list_goals would print out all of the stored goals in goal_map
  void list_goals() {
    if (goal_map.size() == 0)
      std::cout << "No goals available.\n";
    else {
      for (auto it = goal_map.begin(); it != goal_map.end(); it++) {
        std::cout << it->first << " x:" << it->second.x << " y:" << it->second.y
                  << " z:" << it->second.z << " w:" << it->second.w
                  << " id:" << marker_map[it->first] << "\n";
        ;
      }
    }
    main_menu();
  }
  void update_selected_goal() {
    std::shared_ptr<plot> current_node_ptr = shared_plot_from_this();
    auto request = std::make_shared<goal_plotter::srv::Sgoal::Request>();
    request->data = false;
    // wait for the server to be up
    while (!sub_goal_client->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                     "Interrupted while waiting for the service. Exiting.");
        rclcpp::shutdown();
        break;
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
    } else {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"),
                   "Failed to call service Goal not added to goal_map.");
    }
  }
  // This function would add a new goal into the map. With a custom goal_name
  void new_goal() {
    std::cout << "Please enter a name for your goal.\n";
    std::string goal_name;
    std::cin >> goal_name;
    std::cout << "To confirm goal:\n";
    if (wait_confirmation()) {
      update_selected_goal();
      // have intervals of 3. aka m1 = 0, m2=3, m3=6 ...
      // update all maps
      goal_map[goal_name] = selected_goal;
      marker_map[goal_name] = goal_id;
      add_single_marker(visualization_msgs::msg::Marker::ADD, selected_goal,
                        goal_id, goal_name);
      goal_id += 3;
      main_menu();

    } else {
      main_menu();
    }
  }
  // the wait_confirmation function would prompt the user to enter (y\n).
  // Function returns true if (y) and false if (anything else if entered)
  bool wait_confirmation() {
    std::cout << "Please enter (y) for yes. Press and enter anything else "
                 "for no.\n";
    std::string input;
    std::cin >> input;
    return input == "y" ? true : false;
  }

  // add_single_marker function would place a marker on the map at the given
  // pose
  void add_single_marker(uint8_t marker_action, goal_plotter::goal pose,
                         int marker_id, std::string marker_name) {
    auto single_arrow = visualization_msgs::msg::Marker();
    auto single_sphere = visualization_msgs::msg::Marker();
    auto single_text = visualization_msgs::msg::Marker();
    // get current time and fill up the header
    rclcpp::Time time_now = rclcpp::Clock().now();

    single_arrow.header.stamp = time_now;
    single_arrow.header.frame_id = "map";
    single_arrow.id = marker_id;
    single_arrow.type = visualization_msgs::msg::Marker::ARROW;
    single_arrow.action = marker_action;
    single_arrow.pose.position.x = pose.x;
    single_arrow.pose.position.y = pose.y;
    single_arrow.pose.position.z = 0;
    single_arrow.pose.orientation.z = pose.z;
    single_arrow.pose.orientation.w = pose.w;
    single_arrow.scale.x = 0.3;
    single_arrow.scale.y = 0.05;
    single_arrow.scale.z = 0.02;
    single_arrow.color.a = 1.0;
    single_arrow.color.r = 0.0;
    single_arrow.color.g = 250.0;
    single_arrow.color.b = 0.0;

    // add single marker element into the marker array
    marker_array.markers.push_back(single_arrow);

    single_sphere.header.stamp = time_now;
    single_sphere.header.frame_id = "map";
    single_sphere.id = marker_id + 1;
    single_sphere.type = visualization_msgs::msg::Marker::SPHERE;
    single_sphere.action = marker_action;
    single_sphere.pose.position.x = pose.x;
    single_sphere.pose.position.y = pose.y;
    single_sphere.pose.position.z = 0;
    single_sphere.pose.orientation.z = pose.z;
    single_sphere.pose.orientation.w = pose.w;
    single_sphere.scale.x = 0.05;
    single_sphere.scale.y = 0.05;
    single_sphere.scale.z = 0.05;
    single_sphere.color.a = 1.0;
    single_sphere.color.r = 255.0;
    single_sphere.color.g = 0.0;
    single_sphere.color.b = 0.0;

    // add single marker element into the marker array
    marker_array.markers.push_back(single_sphere);

    single_text.header.stamp = time_now;
    single_text.header.frame_id = "map";
    single_text.id = marker_id + 2;
    single_text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    single_text.action = marker_action;
    single_text.pose.position.x = pose.x;
    single_text.pose.position.y = pose.y;
    single_text.pose.position.z = 0;
    single_text.pose.orientation.z = pose.z;
    single_text.pose.orientation.w = pose.w;
    single_text.scale.x = 0.25;
    single_text.scale.y = 0.25;
    single_text.scale.z = 0.25;
    single_text.color.a = 1.0;
    single_text.color.r = 0.0;
    single_text.color.g = 255.0;
    single_text.color.b = 0.0;
    single_text.text = marker_name;

    // add single marker element into the marker array
    marker_array.markers.push_back(single_text);
    marker_array_pub->publish(marker_array);
  }

 private:
  std::map<std::string, goal_plotter::goal> goal_map;
  std::map<std::string, int> marker_map;

  rclcpp::Client<goal_plotter::srv::Sgoal>::SharedPtr sub_goal_client;
  goal_plotter::goal selected_goal;
  int goal_id = 0;

  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_array_pub;

  visualization_msgs::msg::MarkerArray_<std::allocator<void> > marker_array =
      visualization_msgs::msg::MarkerArray();
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor executor;

  std::shared_ptr<plot> plot_ptr = std::make_shared<plot>();
  plot_ptr->main_menu();
  executor.add_node(plot_ptr);

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
