#!/usr/bin/env python3
...

from goal_plotter_messages.msg import Goalactions
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import rclpy
import json
from rclpy.node import Node
from rclpy.action import ActionClient


class ros2_main(Node):
    def __init__(self):
        super().__init__("nav2_goal_manager")
        self.declare_parameter("load_file_path", "null")
        self.goal_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self.goal_file_ = open(self.goal_file_path)
        self.qos_profile_ = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # code to get json dict
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_key_names_ = self.goal_map_.keys()
        print("Loaded waypoints:")
        for x in self.goal_map_:
            print(str(x))
        # self.subscription = self.create_subscription(
        #     Goalactions,
        #     "goal_actions",
        #     self.update_actions,
        #     self.qos_profile_,
        # )
        # self.status_publisher = self.create_publisher(Int32, "goal_state", 10)
        # self.subscription
        # self.goto_pose_ = goto_pose(
        #     ActionClient(self, NavigateToPose, "navigate_to_pose")
        # )


def main(args=None):
    rclpy.init(args=args)

    nav2_goal_manager = ros2_main()

    rclpy.spin(nav2_goal_manager)

    nav2_goal_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
