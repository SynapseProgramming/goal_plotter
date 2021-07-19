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
        self.goal_file_ = open(self.goal_file_path)

        # code to get json dict
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_key_names_ = self.goal_map_.keys()
        print("Loaded waypoints:")
        for x in self.goal_map_:
            print(str(x))


def main(args=None):
    rclpy.init(args=args)
    nav2_goal_manager = ros2_main()

    rclpy.spin(nav2_goal_manager)

    nav2_goal_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
