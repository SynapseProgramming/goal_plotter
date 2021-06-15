#include <cstring>
#include <fstream>
#include <iostream>
#include <map>
#include <string>

#include "goal_plotter/goal.hpp"
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/stringbuffer.h"

// TODO: create a class for the json file reader.

class json_goal_reader {
 public:
  json_goal_reader(std::string full_filepath) {
    full_filepath_ = full_filepath;
  }

  // read_json would convert the goals stored in the json file to a map.
  std::map<std::string, goal_plotter::goal> read_json() {
    std::map<std::string, goal_plotter::goal> obtained_goals;
    std::ifstream read_file(full_filepath_);
    if (read_file.is_open()) {
      rapidjson::IStreamWrapper iswrap(read_file);
      rapidjson::Document document;
      document.ParseStream(iswrap);

      // write the values to obtained_goals
      for (rapidjson::Value::ConstMemberIterator itr = document.MemberBegin();
           itr != document.MemberEnd(); ++itr) {
        // main iterator loop to get the data out
        std::string goal_name = itr->name.GetString();
        goal_plotter::goal goal_pose;
        goal_pose.x = itr->value[0].GetDouble();
        goal_pose.y = itr->value[1].GetDouble();
        goal_pose.z = itr->value[2].GetDouble();
        goal_pose.w = itr->value[3].GetDouble();
        obtained_goals[goal_name] = goal_pose;
      }
      return obtained_goals;
    } else {
      return obtained_goals;
    }
  }

 private:
  std::string full_filepath_;
};

int main() {
  std::string file =
      "/home/ro/dev_ws/src/goal_plotter/goal_json/goal_test.json";

  json_goal_reader test(file);
  std::map<std::string, goal_plotter::goal> goals;
  goals = test.read_json();
  if (goals.size() == 0) {
    std::cout << "ops! There is something wrong.\n";
  } else {
    for (auto it = goals.begin(); it != goals.end(); it++) {
      std::cout << it->first << " " << it->second.x << " " << it->second.y
                << " " << it->second.z << " " << it->second.w << "\n";
    }
  }

  return 0;
}
