#!/usr/bin/env python3
...

import tkinter
import rclpy
import json
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from action_msgs.srv import CancelGoal
from rcl_interfaces.msg import ParameterType


class gui(object):
    def __init__(self, goal_names):

        self.top_ = tkinter.Tk()
        self.goal_names_ = goal_names
        self.top_.geometry("600x500")

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

    def create_button(self, posx, posy, button_name, callback_function, button_colour):
        self.send_goal_button_ = tkinter.Button(
            self.top_, text=button_name, command=callback_function, bg=button_colour
        )
        self.send_goal_button_.place(x=posx, y=posy)


class ros2_main(Node):
    def __init__(self):
        super().__init__("goal_gui")
        self.declare_parameter("load_file_path")
        self.goal_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self.goal_file_ = open(self.goal_file_path)

        # code to get json dict
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_names_ = self.goal_map_.keys()

        self.obj_gui_ = gui(self.goal_names_)
        self.obj_gui_.create_goal_menu()
        self.obj_gui_.create_button(
            posx=100,
            posy=300,
            button_name="send_goal",
            button_colour="green",
            callback_function=self.goal_button_callback,
        )
        self.obj_gui_.create_button(
            posx=100,
            posy=200,
            button_name="cancel_goal",
            button_colour="red",
            callback_function=self.cancel_button_callback,
        )
        self.publisher_ = self.create_publisher(PoseStamped, "/goal_pose", 10)
        self.service_client_ = self.create_client(
            CancelGoal, "navigate_to_pose/_action/cancel_goal"
        )

    def goal_button_callback(self):
        current_goal_name = self.obj_gui_.get_selected_goal()
        if current_goal_name == "goals":
            print("ERROR: NO GOALS SELECTED!")
        else:
            self.current_goal_arr = self.goal_map_[current_goal_name]
            goal_msg_send = self.generate_goal_message(self.current_goal_arr)
            self.publisher_.publish(goal_msg_send)
            self.get_logger().info('Publishing goal: "%s"' % current_goal_name)

    def cancel_button_callback(self):
        print("cancel button pressed")
        while not self.service_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("service not available, waiting again...")
            # Once the server has started, cancel the goal.
        cancel_goal_req = CancelGoal.Request()
        # send the cancel signal
        self.service_client_.call_async(cancel_goal_req)

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
