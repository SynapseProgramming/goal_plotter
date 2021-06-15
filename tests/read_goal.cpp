#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/reader.h"
#include "rapidjson/stringbuffer.h"

int main() {
  rapidjson::Reader reader;
  // MyHandler handler;
  std::ifstream read_file(
      "/home/ro/dev_ws/src/goal_plotter/goal_json/goal_test.json");

  static const char* kTypeNames[] = {"Null",  "False",  "True",  "Object",
                                     "Array", "String", "Number"};

  if (read_file.is_open()) {
    std::cout << "able to read file.\n";

    rapidjson::IStreamWrapper iswrap(read_file);
    // reader.Parse(iswrap, handler);
    rapidjson::Document document;
    document.ParseStream(iswrap);
    for (rapidjson::Value::ConstMemberIterator itr = document.MemberBegin();
         itr != document.MemberEnd(); ++itr) {
      // main iterator loop to get the data out
      printf("Type of member %s is %s  ", itr->name.GetString(),
             kTypeNames[itr->value.GetType()]);
      // print the contents of the array here
      for (rapidjson::SizeType i = 0; i < itr->value.Size(); i++) {
        std::cout << itr->value[i].GetDouble() << " ";
      }
      std::cout << "\n";
    }
  }

  else {
    std::cout << "could not read file!\n";
  }
  return 0;
}
