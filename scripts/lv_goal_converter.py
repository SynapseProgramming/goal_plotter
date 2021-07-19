#!/usr/bin/env python3
...

import json

from rclpy.node import Node
import rclpy


class ros2_main(Node):
    def __init__(self):
        super().__init__("lv_goal_converter")
        self.declare_parameter("load_file_path")
        self.goal_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self.declare_parameter("save_file_path")
        self.save_file_path = (
            self.get_parameter("save_file_path").get_parameter_value().string_value
        )

        self.goal_file_ = open(self.goal_file_path)
        # code to get json dict
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_key_names_ = self.goal_map_.keys()
        self.lv_goal_arr = []
        print("Loaded waypoints:")

    def convert_single_goal(self, goal_name):
        single_goal = {}
        single_goal["goal_name"] = goal_name
        single_goal["goal_pos"] = self.goal_map_[goal_name]
        return single_goal

    def write_to_lv(self):
        print("writing to lv_goal_file")
        for x in self.goal_key_names_:
            self.lv_goal_arr.insert(0, self.convert_single_goal(goal_name=str(x)))

        with open(self.save_file_path, "w") as outfile:
            json.dump(self.lv_goal_arr, outfile)


def main(args=None):
    rclpy.init(args=args)
    lv_goal_converter = ros2_main()
    lv_goal_converter.write_to_lv()

    rclpy.spin(lv_goal_converter)

    lv_goal_converter.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
