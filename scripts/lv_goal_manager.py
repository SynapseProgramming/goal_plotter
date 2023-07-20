#!/usr/bin/env python3
...

from goal_plotter_messages.msg import Goalactions
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
import json
from rclpy.node import Node


class ros2_main(Node):
    def __init__(self):
        super().__init__("lv_goal_manager")
        self.status = Int32()
        self.nav2 = BasicNavigator()
        self.msg = Goalactions()
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
        self.subscription = self.create_subscription(
            Goalactions,
            "goal_actions",
            self.goal_callback,
            self.qos_profile_,
        )
        self.status_publisher = self.create_publisher(Int32, "goal_state", 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def goal_callback(self, m):
        self.msg = m

    def timer_callback(self):
        done = self.nav2.isTaskComplete()
        nav_result = self.nav2.getResult()
        # if the robot has somehow failed(when not idling), then transition the state to 2
        if nav_result == TaskResult.FAILED and done and self.status.data >= 1:
            self.status.data = 2
        # if the robot has reached the goal, reset the status
        elif nav_result == TaskResult.SUCCEEDED and done:
            self.status.data = 0

        # if cancelled button is pressed, reset all goals
        if (
            self.status.data == 1 or self.status.data == 2
        ) and self.msg.cancel_goal == True:
            self.status.data = 0
            self.nav2.cancelTask()

        # if the robot is idling and a new goal has been sent, then move to goal
        if self.status.data == 0 and self.msg.send_goal == True:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.nav2.get_clock().now().to_msg()

            #  if the current goal name is tag, then we will use the goals from the tag
            # TODO: maybe check if the goal is legit or not with some costmap checks
            if self.msg.goal_name == "tag":
                goal_pose.pose.position.x = self.msg.x
                goal_pose.pose.position.y = self.msg.y
                goal_pose.pose.orientation.z = self.msg.z
                goal_pose.pose.orientation.w = self.msg.w
                self.nav2.goToPose(goal_pose)
                self.status.data = 1
            #  otherwise, take it from the current goal list
            elif self.msg.goal_name in self.goal_key_names_:
                current_goal = self.goal_map_[self.msg.goal_name]
                goal_pose.pose.position.x = current_goal[0]
                goal_pose.pose.position.y = current_goal[1]
                goal_pose.pose.orientation.z = current_goal[2]
                goal_pose.pose.orientation.w = current_goal[3]
                self.nav2.goToPose(goal_pose)
                self.status.data = 1
            else:
                # the sent goal does not exist
                self.status.data = 2

        self.status_publisher.publish(self.status)


def main(args=None):
    rclpy.init(args=args)

    nav2_goal_manager = ros2_main()

    rclpy.spin(nav2_goal_manager)

    nav2_goal_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
