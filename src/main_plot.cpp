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

       auto viz_current_goal= geometry_msgs::msg::PoseStamped();
       viz_current_goal.pose.position.x=msg->pose.position.x;
       viz_current_goal.pose.position.y=msg->pose.position.y;
       viz_current_goal.pose.orientation.z=msg->pose.orientation.z;
       viz_current_goal.pose.orientation.w=msg->pose.orientation.w;
       // get current time and fill up the header
        rclcpp::Time time_now = rclcpp::Clock().now();
        viz_current_goal.header.stamp=time_now;
        viz_current_goal.header.frame_id="map";
        selected_goal_pub->publish(viz_current_goal);

    };

    goal_subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose",10,goal_callback);
    selected_goal_pub =this->create_publisher<geometry_msgs::msg::PoseStamped>("selected_goal_pose", 10);
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
    // This function would add a new goal into the map. With a custom goal_name
    void new_goal(){
      std::cout<<"Please enter a name for your goal.\n";
      std::string goal_name;
      std::cin>>goal_name;
      if(wait_confirmation()){


      }
      else main_menu();
      //TODO: complete the new goal function
      //TODO: figure out how to publish a marker for visualisation

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
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr selected_goal_pub;
  goal selected_goal;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plot>());
  rclcpp::shutdown();
  return 0;
}
