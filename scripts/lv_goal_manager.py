#!/usr/bin/env python3
...

from goal_plotter.msg import Goalactions

import rclpy
from rclpy.node import Node


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(
            Goalactions, "goal_actions", 10
        )  # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Goalactions()  # CHANGE
        msg.goal_name = "TEST"
        msg.send_goal = True
        msg.cancel_goal = False
        self.publisher_.publish(msg)
        self.get_logger().info("Publishing!")  # CHANGE


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
