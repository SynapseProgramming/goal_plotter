#include <chrono>
#include <memory>
#include <iostream>
#include <string>
#include <map>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "goal_plotter/goal.h"

#define be RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT


class plot : public rclcpp::Node
{
public:
  plot()
  : Node("goal_plotter")
  {

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
      else{std::cout<<"Please re-enter one of the aforementioned acronyms.\n"; rclcpp::shutdown();}
    }
    // This function would add a new goal into the map. With a custom goal_name
    void new_goal(){
      std::cout<<"Please enter a name for your goal.\n";
      std::string goal_name;
      std::cin>>goal_name;
      if(wait_confirmation()){
       //TODO: when the user confirms that he wants this goal location,  call the service which
       // gets the current selected goal value and stores it inside a map

      }
      else main_menu();
      //TODO: complete the new goal function

    }
    // the wait_confirmation function would prompt the user to enter (y\n). Function returns true if (y) and false if (anything else if entered)
    bool wait_confirmation(){
      std::cout<<"Please enter (y) for yes. Press and enter anything else for no.\n";
      std::string input;
      std::cin >> input;
      return input=="y" ? true: false;
    }
private:
  std::map<std::string,goal_plotter::goal> goal_map;

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<plot>());
  rclcpp::shutdown();
  return 0;
}
