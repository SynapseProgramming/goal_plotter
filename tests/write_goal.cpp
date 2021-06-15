#include <fstream>
#include <iostream>

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

int main() {
  std::ofstream write_file;
  rapidjson::StringBuffer s;
  rapidjson::PrettyWriter<rapidjson::StringBuffer> writer(s);

  writer.StartObject();
  // first goal
  writer.Key("first_goal");
  writer.StartArray();
  double cnt = 0;
  for (unsigned i = 0; i < 4; i++) {
    writer.Double(cnt);
    cnt += 0.1;
  }
  writer.EndArray();
  // second goal
  writer.Key("second_goal");
  writer.StartArray();
  double cnt_2 = 2.0;
  for (unsigned i = 0; i < 4; i++) {
    writer.Double(cnt_2);
    cnt_2 += 0.1;
  }
  writer.EndArray();

  writer.EndObject();
  // writer.EndObject();
  std::cout << s.GetString() << std::endl;

  write_file.open("/home/ro/dev_ws/src/goal_plotter/goal_json/goal_test.json");
  if (write_file.is_open()) {
    write_file << s.GetString();
    write_file.close();
  } else {
    std::cout << "something went wrong. Unable to open file.\n";
  }
  return 0;
}
