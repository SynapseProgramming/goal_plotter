#!/usr/bin/env python3
...

from goal_plotter.msg import Goalactions

import rclpy
from rclpy.node import Node


class ros2_main(Node):
    def __init__(self):
        super().__init__("nav2_goal_manager")
        self.goal_name_ = str()
        self.send_goal_ = False
        self.cancel_goal_ = False
        self.subscription = self.create_subscription(
            Goalactions, "goal_actions", self.update_actions, 10
        )
        self.subscription

    def update_actions(self, msg):
        print(str(self.goal_name_))
        print(str(self.send_goal_))
        print(str(self.cancel_goal_))
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
