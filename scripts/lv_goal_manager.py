#!/usr/bin/env python3
...

from goal_plotter.msg import Goalactions
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32

import rclpy
import json
from rclpy.node import Node


class ros2_main(Node):
    def __init__(self):
        super().__init__("nav2_goal_manager")
        self.goal_name_ = str()
        self.send_goal_ = False
        self.cancel_goal_ = False
        self.current_state_ = 0
        self.timer = self.create_timer(0.1, self.timed_callback)
        self.declare_parameter("load_file_path")
        self.goal_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self.goal_file_ = open(self.goal_file_path)

        # code to get json dict
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_names_ = self.goal_map_.keys()
        print("Loaded waypoints:")
        for x in self.goal_map_:
            print(str(x))
        self.subscription = self.create_subscription(
            Goalactions, "goal_actions", self.update_actions, 10
        )
        self.status_publisher = self.create_publisher(Int32, "goal_state", 10)
        self.subscription

    def timed_callback(self):
        # send a goal to nav stack when the send_goal is true and there is a valid goal
        if (
            (self.goal_name_ in self.goal_map_)
            and self.current_state_ == 0
            and self.send_goal_ == True
        ):
            print("STONKS")
            self.current_state_ = 1
        self.pub_status(self.current_state_)

    def pub_status(self, stat):
        msg = Int32()
        msg.data = stat
        self.status_publisher.publish(msg)

    def update_actions(self, msg):
        self.goal_name_ = msg.goal_name
        self.send_goal_ = msg.send_goal
        self.cancel_goal_ = msg.cancel_goal


def main(args=None):
    rclpy.init(args=args)

    nav2_goal_manager = ros2_main()

    rclpy.spin(nav2_goal_manager)

    nav2_goal_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
