#include <cstring>
#include <fstream>
#include <iostream>
#include <string>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/reader.h"
#include "rapidjson/stringbuffer.h"

/*
struct MyHandler {
  bool Null() {
    std::cout << "Null()" << std::endl;
    return true;
  }
  bool Bool(bool b) {
    std::cout << "Bool(" << std::boolalpha << b << ")" << std::endl;

    return true;
  }
  bool Int(int i) {
    std::cout << "Int(" << i << ")" << std::endl;
    return true;
  }
  bool Uint(unsigned u) {
    std::cout << "Uint(" << u << ")" << std::endl;
    return true;
  }
  bool Int64(int64_t i) {
    std::cout << "Int64(" << i << ")" << std::endl;
    return true;
  }
  bool Uint64(uint64_t u) {
    std::cout << "Uint64(" << u << ")" << std::endl;
    return true;
  }
  bool Double(double d) {
    std::cout << "Double(" << d << ")" << std::endl;
    return true;
  }
  bool RawNumber(const char* str, rapidjson::SizeType length, bool copy) {
    std::cout << "Number(" << str << ", " << length << ", " << std::boolalpha
              << copy << ")" << std::endl;
    return true;
  }
  bool String(const char* str, rapidjson::SizeType length, bool copy) {
    std::cout << "String(" << str << ", " << length << ", " << std::boolalpha
              << copy << ")" << std::endl;
    return true;
  }
  bool StartObject() {
    std::cout << "StartObject()" << std::endl;
    return true;
  }
  bool Key(const char* str, rapidjson::SizeType length, bool copy) {
    std::cout << "Key(" << str << ", " << length << ", " << std::boolalpha
              << copy << ")" << std::endl;
    return true;
  }
  bool EndObject(rapidjson::SizeType memberCount) {
    std::cout << "EndObject(" << memberCount << ")" << std::endl;
    return true;
  }
  bool StartArray() {
    std::cout << "StartArray()" << std::endl;
    return true;
  }
  bool EndArray(rapidjson::SizeType elementCount) {
    std::cout << "EndArray(" << elementCount << ")" << std::endl;
    return true;
  }
};
*/

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

    /*
    std::string input = "{ \"test\": true }";

    char json[input.length() + 1];
    std::strcpy(json, input.c_str());
    rapidjson::StringStream ss(json);
    reader.Parse(ss, handler);*/
  }

  else {
    std::cout << "could not read file!\n";
  }
  return 0;
}
