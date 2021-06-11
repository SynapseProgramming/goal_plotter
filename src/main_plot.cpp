#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT

struct goal{
    double x;
    double y;
    double z;
    double w;
};



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

    goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",10,goal_callback);
    main_menu();
  }
    void main_menu(){
      std::string input;
      std::cout<<"(ag) Add a new goal\n";
      std::cout<<"(ls) List all goals\n";
      std::cout<<"(rm) Remove a specific goal\n";
      std::cout<<"(ck) Check if a goal exists\n";
      std::cout<<"(sv) Save and export goal file\n";

      std::cin>>input;
      if(input=="ag"){new_goal();}
      else{std::cout<<"Please re-enter one of the aforementioned acronyms.\n";main_menu();}
    }
    void new_goal(){
      std::cout<<"Please enter a name for your goal.\n";
      std::string goal_name;
      std::cin>>goal_name;
      if(wait_confirmation()) std::cout<<goal_name<<"\n";
      else main_menu();

    }
    // the wait_confirmation function would prompt the user to enter (y\n). Function returns true if (y) and false if (anything else if entered)
    bool wait_confirmation(){
      std::cout<<"Please enter (y) for yes. Press and enter anything else for no.\n";
      std::string input;
      std::cin >> input;
      return input=="y" ? true: false;
    }
private:
  std::map<std::string,goal> goal_map;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_subscriber;


};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plot>());
  rclcpp::shutdown();
  return 0;
}
