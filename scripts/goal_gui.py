#!/usr/bin/env python3
...

import tkinter  # note that module name has changed from Tkinter in Python 2 to tkinter in Python 3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# in python, calling a function without any brackets would = calling the function by reference ( giving the functions address)
class gui(object):
    def __init__(self, goal_map):

        self.top_ = tkinter.Tk()
        self.goal_map_ = goal_map
        self.top_.geometry("700x800")
        self.create_goal_menu()

    def create_goal_menu(self):
        self.selected_goal_ = tkinter.StringVar(self.top_)
        self.selected_goal_.set(self.goal_map_[0])
        self.goal_menu = tkinter.OptionMenu(
            self.top_, self.selected_goal_, *self.goal_map_
        )
        self.goal_menu.place(x=400, y=300)

        # callback_function is the address of the function to callback when the button is pressed.

    def create_goal_button(self, callback_function):
        self.send_goal_button_ = tkinter.Button(
            self.top_, text="send_goal", command=callback_function
        )
        self.send_goal_button_.place(x=100, y=200)


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__("minimal_publisher")
        GOALS = ["g1", "g2", "g3"]
        self.obj_gui_ = gui(GOALS)
        self.obj_gui_.create_goal_button(self.button_callback)
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.i = 0

    def button_callback(self):
        msg = String()
        msg.data = "Hello World: %d" % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    tkinter.mainloop()

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
