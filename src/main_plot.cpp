#include <chrono>
#include <memory>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

class plot : public rclcpp::Node
{
public:

  plot()
  : Node("goal_plotter")
  {
     auto goal_callback= [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){
       std::cout<<"goal received: "<<msg->pose.position.x<<" "<<msg->pose.position.y<<" ";
       std::cout<<msg->pose.orientation.z<<" "<<msg->pose.orientation.w<<"\n";
    };

    //publisher_ = this->create_publisher<std_msgs::msg::Int32>("result", 10);
    subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",10,goal_callback);

  }
private:

  //shared ptr of a timer
  //rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plot>());
  rclcpp::shutdown();
  return 0;
}
