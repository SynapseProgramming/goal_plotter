#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "goal_plotter/goal.h"



class get_goal : public rclcpp::Node
{
public:
  get_goal():
    Node("sub_goal_node")
  {
    selected_goal.x=0;
    selected_goal.y=0;
    selected_goal.z=0;
    selected_goal.w=0;

    auto goal_callback= [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
       std::cout<<"goal received: "<<msg->pose.position.x<<" "<<msg->pose.position.y<<" ";
       std::cout<<msg->pose.orientation.z<<" "<<msg->pose.orientation.w<<"\n";
       selected_goal.x=msg->pose.position.x;
       selected_goal.y=msg->pose.position.y;
       selected_goal.z=msg->pose.orientation.z;
       selected_goal.w=msg->pose.orientation.w;
       //TODO: create a service which would return the current selected goal value
    };

    goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",10,goal_callback);
  }
private:

rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;
goal_plotter::goal selected_goal;

};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<get_goal>());
  rclcpp::shutdown();
  return 0;
}
