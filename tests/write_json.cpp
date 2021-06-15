#include <iostream>

#include "rapidjson/prettywriter.h"
#include "rapidjson/stringbuffer.h"
#include "rapidjson/writer.h"

using namespace rapidjson;
using namespace std;

int main() {
  StringBuffer s;
  PrettyWriter<StringBuffer> writer(s);

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

  cout << s.GetString() << endl;
  return 0;
}
