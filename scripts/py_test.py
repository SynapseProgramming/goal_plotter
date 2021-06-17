#!/usr/bin/env python3
...

import tkinter  # note that module name has changed from Tkinter in Python 2 to tkinter in Python 3


top = tkinter.Tk()
top.geometry("700x800")

GOALS = ["g1", "g2", "g3"]

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
top.mainloop()
