#!/usr/bin/python3

import rospy
import numpy
import math
import actionlib
import tf2_ros
import tf_conversions
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseArray, Point
from action_client.msg import GoToAction, GoToGoal
from std_msgs.msg import Float32

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("waypoint_unity", anonymous=True)

        ## | --------------------- load parameters -------------------- |
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.drone_list = rospy.get_param('~uav_names', [])
        self.computer_name = rospy.get_param("~computer_name", "computer1")
        self.human_name = rospy.get_param("~human_name", "human")

        self.leader_name = self.drone_list[0]

        self.leader_swarm = False
        if self.uav_name == self.leader_name:
            self.leader_swarm = True
        
        self.id = 0
        for name in self.drone_list:
            if name == self.uav_name: break
            self.id += 1
        rospy.loginfo("ID: %d", self.id)

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.timer_human_rate = rospy.get_param("~timer_human/rate")
        self.timer_point_rate = rospy.get_param("~timer_point/rate")
        self.timer_heading_rate = rospy.get_param("~timer_heading/rate")

        self.index = rospy.get_param("~index")
        self.side = rospy.get_param("~side")
        self.human_distance_threshold = rospy.get_param("~human_distance_threshold", 10.0)

        ## | --------------------- variables -------------------- |
        self.feedback_value = 0
        self.waypoint_initialized = False

        ## | --------------------- Action clients -------------------- |
        if self.leader_swarm:
            self.action_name = f"/uav/{self.uav_name}/action/GoTo"
            self.goto_client = actionlib.SimpleActionClient(
                self.action_name, GoToAction
            )

        ## | ------------------------- timers ------------------------- |
        if self.leader_swarm:
            # Waypoint Timer
            self.timer_waypoint = rospy.Timer(rospy.Duration(1.0/self.timer_human_rate), self.timerWaypoint)

        ## | -------------------- spin till the end ------------------- |
        rospy.loginfo('[Waypoint_Unity]: initialized')
        self.is_initialized = True
        rospy.spin()

    # #} end of __init__()


    ## | ------------------- Action callbacks ------------------- |

    def goto_done_cb(self, state, result):
        rospy.loginfo(f"[{self.uav_name}] state: {state}")
        rospy.loginfo(f"[{self.uav_name}] GoTo finished with result: {result.result}")

    def goto_active_cb(self):
        rospy.loginfo_once(f"[{self.uav_name}] GoTo goal is now active.")

    def goto_feedback_cb(self, feedback):
        rospy.loginfo(f"[{self.uav_name}] Feedback: {feedback.feedback:.2f}")

    ## | ------------------------- timers ------------------------- |

    # #{ timerWaypoint()
    def timerWaypoint(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[Waypoint_Unity]: waypoint timer spinning')

        if self.waypoint_initialized == False:
            self.waypoint_initialized = True

            waypoints = [
                [30, 0, 6.0],
                [30, 280, 6.0],
                [-240, 280, 6.0],
                [-240, 0, 6.0],
                [30, 0, 6.0]
            ]
            for i in range(len(waypoints)):
                goal_msg = GoToGoal()
                goal_msg.target.x = waypoints[i][0]
                goal_msg.target.y = waypoints[i][1]
                goal_msg.target.z = waypoints[i][2]

                self.goto_client.send_goal(goal_msg, done_cb=self.goto_done_cb, active_cb=self.goto_active_cb, feedback_cb=self.goto_feedback_cb)
                rospy.loginfo(f"[{self.uav_name}] Waypoint sent: {goal_msg.target.x}, {goal_msg.target.y}, {goal_msg.target.z}")
                self.goto_client.wait_for_result()
                rospy.sleep(2.0)

    # #} end of timerWaypoint()


if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
