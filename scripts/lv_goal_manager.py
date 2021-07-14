#!/usr/bin/env python3
...

from goal_plotter.msg import Goalactions
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Int32

import rclpy
import json
from rclpy.node import Node
from rclpy.action import ActionClient


class goto_pose:
    def __init__(self, action_client):
        self._action_client = action_client
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            print("nav2 Goal Rejected")
            self.goal_accept_status = False
            return

        print("nav2 Goal accepted")
        self.goal_accept_status = True

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):

        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            print("navigation succeeded")
            self.goal_status = True
        else:
            print("navigation failed!")
            self.goal_status = False
            self.failure_flag = True

        # send_goal would accept an array as a goal pose

    def send_goal(self, goal_pose):
        print("Waiting for action server")
        self._action_client.wait_for_server()
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.pose.position.x = goal_pose[0]
        goal_msg.pose.pose.position.y = goal_pose[1]
        goal_msg.pose.pose.orientation.z = goal_pose[2]
        goal_msg.pose.pose.orientation.w = goal_pose[3]
        # fill up the rest later
        print("Sending goal request")

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def get_status(self):
        status = {}
        status["gs"] = self.goal_status
        status["gas"] = self.goal_accept_status
        status["f_flag"] = self.failure_flag
        return status

    def reset_status(self):
        self.goal_status = False
        self.goal_accept_status = False
        self.failure_flag = False


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
        self.goto_pose_ = goto_pose(
            ActionClient(self, NavigateToPose, "navigate_to_pose")
        )

    def timed_callback(self):
        # update statuses
        nav2_status = self.goto_pose_.get_status()
        # send a goal to nav stack when the send_goal is true and there is a valid goal
        if (
            (self.goal_name_ in self.goal_map_)
            and self.current_state_ == 0
            and self.send_goal_ == True
        ):
            self.goto_pose_.send_goal(self.goal_map_[self.goal_name_])
            self.current_state_ = 1
        # once the robot has reached the goal, reset state back to 0
        elif (
            self.current_state_ == 1
            and nav2_status["gs"] == True
            and nav2_status["gas"] == True
        ):
            print("Goal Reached!")
            self.current_state_ = 0
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
