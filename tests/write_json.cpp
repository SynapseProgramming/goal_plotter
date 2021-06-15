#include <iostream>

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

using namespace rapidjson;
using namespace std;

int main() {
  StringBuffer s;
  PrettyWriter<StringBuffer> writer(s);
  // uncomment this block to test out the basic functionalities of the writer
  // function
  /*
    writer.StartObject();
    writer.Key("hello");
    writer.String("world");
    writer.Key("t");
    writer.Bool(true);
    writer.Key("f");
    writer.Bool(false);
    writer.Key("n");
    writer.Null();
    writer.Key("i");
    writer.Uint(123);
    writer.Key("pi");
    writer.Double(3.1416);
    writer.Key("a");
    writer.StartArray();
    for (unsigned i = 0; i < 4; i++) writer.Uint(i);
    writer.EndArray();
    writer.EndObject();
  */

  // we will use an array to store data double array test (key=goal name,
  // array[0]=x,[1]=y,[2]=z,[3]=w) nested object output test
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
  cout << s.GetString() << endl;
  return 0;
}
