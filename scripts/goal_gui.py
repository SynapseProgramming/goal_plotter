#!/usr/bin/env python3
...

import tkinter
import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterType


test_file_path = (
    "/home/ro/dev_ws/install/goal_plotter/share/goal_plotter/goal_json/goal_test.json"
)


class gui(object):
    def __init__(self, goal_names):

        self.top_ = tkinter.Tk()
        self.goal_names_ = goal_names
        self.top_.geometry("700x800")

    def create_goal_menu(self):
        self.selected_goal_ = tkinter.StringVar(self.top_)
        self.selected_goal_.set("goals")
        self.goal_menu = tkinter.OptionMenu(
            self.top_, self.selected_goal_, *self.goal_names_
        )
        self.goal_menu.place(x=400, y=300)
        # get selected_goal would return the name of the selected goal.

    def get_selected_goal(self):
        return self.selected_goal_.get()

    # callback_function is the address of the function to callback when the button is pressed.

    def create_goal_button(self, callback_function):
        self.send_goal_button_ = tkinter.Button(
            self.top_, text="send_goal", command=callback_function
        )
        self.send_goal_button_.place(x=100, y=300)


class ros2_main(Node):
    def __init__(self):
        super().__init__("goal_gui")
        self.declare_parameter("load_file_path", test_file_path)
        self.goal_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self.goal_file_ = open(self.goal_file_path)

        # code to get json dict
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_names_ = self.goal_map_.keys()

        self.obj_gui_ = gui(self.goal_names_)
        self.obj_gui_.create_goal_menu()
        self.obj_gui_.create_goal_button(self.button_callback)
        self.publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)

    def button_callback(self):
        current_goal_name = self.obj_gui_.get_selected_goal()
        self.current_goal_arr = self.goal_map_[current_goal_name]
        goal_msg_send = self.generate_goal_message(self.current_goal_arr)
        self.publisher_.publish(goal_msg_send)
        self.get_logger().info('Publishing goal: "%s"' % current_goal_name)

    def generate_goal_message(self, goal_array):
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = goal_array[0]
        goal_msg.pose.position.y = goal_array[1]
        goal_msg.pose.orientation.z = goal_array[2]
        goal_msg.pose.orientation.w = goal_array[3]

        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = "map"
        return goal_msg


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ros2_main()

    tkinter.mainloop()

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
