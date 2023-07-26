# goal_plotter
Program which allows users to plot goals on a map and save the goals to a json file.

## Dependencies
- Install xterm:
  ```
  sudo apt install xterm
  ```

## Installation

- git clone the goal_plotter_messages package into your ros directory <br/>
  ```
  git clone https://github.com/SynapseProgramming/goal_plotter_messages.git
  ```
- git clone this package into your ros directory <br/>
  ```
  git clone https://github.com/SynapseProgramming/goal_plotter.git
  ```

- run `colcon build`

## Usage


- In `goal_plotter.launch.py` please change `map_name` to the name of your map. <br />
- Next, change file path of `map_yaml_file` if needed. <br />
- Next, enter a unique name for `save_goal_file` and `load_goal_file`. save_goal_file will be generated in `<your_workspace>/install/goal_plotter/share/goal_plotter/goal_json`  <br />
- Next, in `goal_gui.launch.py` Please change `load_goal_file` if needed. <br />
- Run `colcon build` and run `ros2 launch goal_plotter goal_plotter.launch.py` to launch the plotter program.<br />
- Run `ros2 launch goal_plotter goal_gui.launch` to open a GUI interface to send goals to the navigation stack.


