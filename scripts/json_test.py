#!/usr/bin/env python3
...

import json

goal_file = open(
    "/home/ro/dev_ws/install/goal_plotter/share/goal_plotter/goal_json/goal_test.json"
)

goal_map = json.load(goal_file)


for goal_name, goal_location in goal_map.items():
    print(goal_name)
    gx = goal_location[0]
    gy = goal_location[1]
    gz = goal_location[2]
    gw = goal_location[3]
    print(gx, gy, gz, gw)
