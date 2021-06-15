#include <fstream>
#include <iostream>

#include "goal_plotter/goal.hpp"
#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

class json_goal_writer {
 public:
  json_goal_writer(std::string full_filepath) : writer(s) {
    full_filepath_ = full_filepath;
  }
  // this function would initialise the json_goal writer and open the file
  // mentioned in the constructor
  bool begin_write() {
    write_file.open(full_filepath_);
    if (write_file.is_open()) {
      writer.StartObject();
      return true;
    } else {
      return false;
    }
  }
  // This function would write an array to the json writer
  void write_array(std::string goal_name, goal_plotter::goal goal_pose) {
    writer.Key(goal_name.c_str());
    writer.StartArray();
    // [0]x [1]y [2]z [3]w
    writer.Double(goal_pose.x);
    writer.Double(goal_pose.y);
    writer.Double(goal_pose.z);
    writer.Double(goal_pose.w);
    writer.EndArray();
  }
  // this function would save the goal poses to the json file
  void stop_write() {
    writer.EndObject();
    write_file << s.GetString();
    write_file.close();
  }

 private:
  std::ofstream write_file;
  rapidjson::StringBuffer s;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer;
  std::string full_filepath_;
};

int main() {
  goal_plotter::goal pose;
  pose.x = 1.0;
  pose.y = 2.0;
  pose.z = 3.0;
  pose.w = 4.0;

  json_goal_writer test(
      "/home/ro/dev_ws/src/goal_plotter/goal_json/goal_test.json");
  if (test.begin_write()) {
    test.write_array("HEHE", pose);
    pose.x = 10.0;
    pose.y = 12.0;
    pose.z = 13.0;
    pose.w = 14.0;
    test.write_array("LOL", pose);

    test.stop_write();
  }

  return 0;
}
