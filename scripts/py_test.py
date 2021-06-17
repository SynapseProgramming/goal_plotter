#!/usr/bin/env python3
...

import tkinter  # note that module name has changed from Tkinter in Python 2 to tkinter in Python 3

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
        self.create_goal_button()

    def create_goal_button(self):
        self.send_goal_button_ = tkinter.Button(
            self.top_, text="send_goal", command=self.send_goal_callback
        )
        self.send_goal_button_.place(x=100, y=200)

    def send_goal_callback(self):
        print("callback called!")
        print(self.selected_goal_.get())


GOALS = ["g1", "g2", "g3"]
obj_gui = gui(GOALS)
obj_gui.create_goal_menu()
# obj_gui.create_goal_button()
tkinter.mainloop()

"""
top = tkinter.Tk()
top.geometry("700x800")


# selected goal is a string used to store the currenty selected drop down
selected_goal = tkinter.StringVar(top)
selected_goal.set(GOALS[0])

# create options menu widget
goal_menu = tkinter.OptionMenu(top, selected_goal, *GOALS)
goal_menu.place(x=400, y=300)


def send_goal_callback():
    print(selected_goal.get())


send_goal_button = tkinter.Button(top, text="send_goal", command=send_goal_callback)
send_goal_button.place(x=100, y=200)
"""
