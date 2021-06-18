#!/usr/bin/env python3
...

import tkinter  # note that module name has changed from Tkinter in Python 2 to tkinter in Python 3
import rclpy
import json
from rclpy.node import Node
from std_msgs.msg import String

goal_file = open(
    "/home/ro/dev_ws/install/goal_plotter/share/goal_plotter/goal_json/goal_test.json"
)


# in python, calling a function without any brackets would = calling the function by reference ( giving the functions address)
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
        super().__init__("minimal_publisher")
        # code to get json dict
        self.goal_file_ = goal_file
        self.goal_map_ = json.load(self.goal_file_)
        self.goal_names_ = self.goal_map_.keys()

        self.obj_gui_ = gui(self.goal_names_)
        self.obj_gui_.create_goal_menu()
        self.obj_gui_.create_goal_button(self.button_callback)
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.i = 0

    def button_callback(self):
        msg = String()
        print(self.obj_gui_.get_selected_goal())
        # print(self.goal_map_[self.obj_gui_.get_selected_goal()])
        self.current_goal_arr = self.goal_map_[self.obj_gui_.get_selected_goal()]
        self.cx = self.current_goal_arr[0]
        self.cy = self.current_goal_arr[1]
        self.cz = self.current_goal_arr[2]
        self.cw = self.current_goal_arr[3]
        print(self.cx, self.cy, self.cz, self.cw)
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = ros2_main()

    tkinter.mainloop()

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
