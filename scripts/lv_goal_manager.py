#!/usr/bin/env python3
...

from goal_plotter_messages.msg import Goalactions
from std_msgs.msg import Int32
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, PolygonStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.msg import Costmap
from nav_msgs.msg import OccupancyGrid
from footprint_collision_checker import FootprintCollisionChecker
from costmap_2d import PyCostmap2D
from footprint_generator import FootprintGenerator
from visualization_msgs.msg import Marker
import rclpy
import json
import numpy as np
from rclpy.node import Node


class ros2_main(Node):
    def __init__(self):
        super().__init__("lv_goal_manager")

        self.declare_parameter("load_file_path", "null")
        self.declare_parameter("robot_radius", 0.5)
        self.declare_parameter("reject_cost", 254)
        self.goal_file_path = (
            self.get_parameter("load_file_path").get_parameter_value().string_value
        )
        self.robot_radius = (
            self.get_parameter("robot_radius").get_parameter_value().double_value
        )

        self.reject_cost = (
            self.get_parameter("reject_cost").get_parameter_value().integer_value
        )

        self.status = Int32()
        self.nav2 = BasicNavigator()
        self.costOccupancyGrid = OccupancyGrid()
        self.footprintChecker = FootprintCollisionChecker()
        self.footprintgen = FootprintGenerator(self.robot_radius)
        self.tagfootprint = PolygonStamped()
        self.markerdir = Marker()

        self.markerdir.pose.orientation.z = 1.0
        self.markerdir.scale.x = 0.3
        self.markerdir.scale.y = 0.05
        self.markerdir.scale.z = 0.02
        self.markerdir.color.a = 1.0
        self.markerdir.color.r = 0.0
        self.markerdir.color.g = 0.0
        self.markerdir.color.b = 250.0
        self.markerdir.header.frame_id = "map"
        self.markerdir.type = 0

        self.tagfootprint.header.frame_id = "map"
        self.tagfootprint.header.stamp = self.nav2.get_clock().now().to_msg()
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
        self.tagoal_publisher = self.create_publisher(PolygonStamped, "tag_pose", 10)
        self.arrow_publisher = self.create_publisher(Marker, "tag_direction", 10)

    #    function to get the latest global costmap from nav2
    def get_globalcostmap(self):
        nav2Costmap = self.nav2.getGlobalCostmap()
        # update values
        self.costOccupancyGrid.header = nav2Costmap.header
        self.costOccupancyGrid.data = np.array(nav2Costmap.data, dtype="i1").tolist()

        self.costOccupancyGrid.info.map_load_time = nav2Costmap.metadata.map_load_time
        self.costOccupancyGrid.info.resolution = nav2Costmap.metadata.resolution
        self.costOccupancyGrid.info.width = nav2Costmap.metadata.size_x
        self.costOccupancyGrid.info.height = nav2Costmap.metadata.size_y
        self.costOccupancyGrid.info.origin = nav2Costmap.metadata.origin

    def goal_callback(self, msg):
        done = self.nav2.isTaskComplete()
        nav_result = self.nav2.getResult()

        q_error = abs(msg.z * msg.z + msg.w * msg.w - 1.0)

        self.footprintgen.setPose(msg.x, msg.y)
        bot_footprint = self.footprintgen.getFootprint()

        # update visualization
        self.tagfootprint.polygon = bot_footprint
        self.tagfootprint.header.stamp = self.nav2.get_clock().now().to_msg()

        self.markerdir.pose.position.x = msg.x
        self.markerdir.pose.position.y = msg.y
        if q_error <= 0.01:
            self.markerdir.pose.orientation.z = msg.z
            self.markerdir.pose.orientation.w = msg.w

        # if the robot has somehow failed(when not idling), then transition the state to 2
        if nav_result == TaskResult.FAILED and done and self.status.data >= 1:
            self.status.data = 2
        # if the robot has reached the goal, reset the status
        elif nav_result == TaskResult.SUCCEEDED and done:
            self.status.data = 0

        # if cancelled button is pressed, reset all goals
        if (self.status.data == 1 or self.status.data == 2) and msg.cancel_goal == True:
            self.status.data = 0
            self.nav2.cancelTask()

        # if the robot is idling and a new goal has been sent, then move to goal
        if self.status.data == 0 and msg.send_goal == True:
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = self.nav2.get_clock().now().to_msg()

            #  if the current goal name is tag, then we will use the goals from the tag
            if msg.goal_name == "tag":
                self.get_globalcostmap()
                latestglobal = PyCostmap2D(self.costOccupancyGrid)
                self.footprintChecker.setCostmap(latestglobal)

                #   get the cost of the footprint
                cost = self.footprintChecker.footprintCost(bot_footprint)

                print("dynamic goal cost: " + str(cost))
                if cost < self.reject_cost and q_error <= 0.01:
                    goal_pose.pose.position.x = msg.x
                    goal_pose.pose.position.y = msg.y
                    goal_pose.pose.orientation.z = msg.z
                    goal_pose.pose.orientation.w = msg.w
                    self.nav2.goToPose(goal_pose)
                    self.status.data = 1
                else:
                    self.status.data = 2
            #  otherwise, take it from the current goal list
            elif msg.goal_name in self.goal_key_names_:
                current_goal = self.goal_map_[msg.goal_name]
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
        self.tagoal_publisher.publish(self.tagfootprint)
        self.arrow_publisher.publish(self.markerdir)


def main(args=None):
    rclpy.init(args=args)

    nav2_goal_manager = ros2_main()

    rclpy.spin(nav2_goal_manager)

    nav2_goal_manager.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
