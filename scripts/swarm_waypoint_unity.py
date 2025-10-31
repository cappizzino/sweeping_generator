#!/usr/bin/python3

import rospy
import actionlib
from action_client.msg import GoToAction, GoToGoal
from std_msgs.msg import Empty
import numpy as np

BLACK = "\033[30m"
RED = "\033[31m"
GREEN = "\033[32m"
YELLOW = "\033[33m"
BLUE = "\033[34m"
MAGENTA = "\033[35m"
CYAN = "\033[36m"
WHITE = "\033[37m"

# ┌─────────────────────────────────────┐
# │ 2. Bright (High-Intensity) Foreground │
# └─────────────────────────────────────┘
BRIGHT_BLACK = "\033[90m"
BRIGHT_RED = "\033[91m"
BRIGHT_GREEN = "\033[92m"
BRIGHT_YELLOW = "\033[93m"
BRIGHT_BLUE = "\033[94m"
BRIGHT_MAGENTA = "\033[95m"
BRIGHT_CYAN = "\033[96m"
BRIGHT_WHITE = "\033[97m"

# ┌───────────────────────────────┐
# │ 3. Standard Backgrounds      │
# └───────────────────────────────┘
BG_BLACK = "\033[40m"
BG_RED = "\033[41m"
BG_GREEN = "\033[42m"
BG_YELLOW = "\033[43m"
BG_BLUE = "\033[44m"
BG_MAGENTA = "\033[45m"
BG_CYAN = "\033[46m"
BG_WHITE = "\033[47m"

# ┌────────────────────────────────┐
# │ 4. Bright Backgrounds         │
# └────────────────────────────────┘
BG_BRIGHT_BLACK = "\033[100m"
BG_BRIGHT_RED = "\033[101m"
BG_BRIGHT_GREEN = "\033[102m"
BG_BRIGHT_YELLOW = "\033[103m"
BG_BRIGHT_BLUE = "\033[104m"
BG_BRIGHT_MAGENTA = "\033[105m"
BG_BRIGHT_CYAN = "\033[106m"
BG_BRIGHT_WHITE = "\033[107m"

# ┌───────────────────────────────┐
# │ 5. Text Styles & Resets      │
# └───────────────────────────────┘
RESET_ALL = "\033[0m"   # resets everything to default
BOLD = "\033[1m"
DIM = "\033[2m"
ITALIC = "\033[3m"   # not supported in all terminals
UNDERLINE = "\033[4m"
BLINK = "\033[5m"
INVERT = "\033[7m"   # swap fg/bg
HIDDEN = "\033[8m"
STRIKETHROUGH = "\033[9m"

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("waypoint_unity", anonymous=True)

        ## | --------------------- load parameters -------------------- |
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.drone_list = rospy.get_param('~uav_names', [])
        self.human_name = rospy.get_param("~human_name", "human")
        self.step_size = rospy.get_param("~step_size", 40.0)

        # Determine leader name
        self.leader_name = self.drone_list[0]

        self.leader_swarm = False
        if self.uav_name == self.leader_name:
            self.leader_swarm = True

        # Determine UAV ID
        self.id = 0
        for name in self.drone_list:
            if name == self.uav_name: break
            self.id += 1
        rospy.loginfo("ID: %d", self.id)

        self.world_frame = rospy.get_param("~world_frame", "world")
        self.timer_waypoint_rate = rospy.get_param("~timer_waypoint/rate")

        ## | --------------------- variables -------------------- |
        self.feedback_value = 0
        self.waypoint_initialized = False

        ## | --------------------- Publisher -------------------- |
        self.stop_pub = rospy.Publisher("/stop_measurement", Empty, queue_size=10)

        ## | --------------------- Action clients -------------------- |
        if self.leader_swarm:
            self.action_name = f"/uav/{self.uav_name}/action/GoTo"
            self.goto_client = actionlib.SimpleActionClient(
                self.action_name, GoToAction
            )

        ## | ------------------------- timers ------------------------- |
        if self.leader_swarm:
            # Waypoint Timer
            self.timer_waypoint = rospy.Timer(rospy.Duration(1.0/self.timer_waypoint_rate), self.timerWaypoint)

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

        # rospy.loginfo_once('[Waypoint_Unity]: waypoint timer spinning')
        rospy.loginfo_once(f"{BG_BRIGHT_CYAN}[Waypoint_Unity]: waypoint timer spinning{RESET_ALL}")

        if self.waypoint_initialized == False:
            self.waypoint_initialized = True

            waypoints = [
                [20, 0, 2.0],
                [20, 280, 2.0],
                [-240, 280, 2.0],
                [-240, 0, 2.0],
                [20, 0, 2.0]
            ]
            trajectory_points = self.generate_trajectory_steps(waypoints, self.step_size)

            for i in range(len(trajectory_points)):
                goal_msg = GoToGoal()
                goal_msg.target.x = trajectory_points[i][0]
                goal_msg.target.y = trajectory_points[i][1]
                goal_msg.target.z = trajectory_points[i][2]

                self.goto_client.send_goal(goal_msg, done_cb=self.goto_done_cb, active_cb=self.goto_active_cb, feedback_cb=self.goto_feedback_cb)
                rospy.loginfo(f"{BG_BRIGHT_CYAN} [{self.uav_name}] Waypoint sent: {RESET_ALL} {goal_msg.target.x}, {goal_msg.target.y}, {goal_msg.target.z}")
                self.goto_client.wait_for_result()
                rospy.sleep(0.1)

            self.stop_pub.publish(Empty())
            rospy.loginfo(f"{BG_BRIGHT_CYAN}[{self.uav_name}] All waypoints completed. Stop measurement signal sent.{RESET_ALL}")

    # #} end of timerWaypoint()

    ## | ------------------------- methods ------------------------- |

    # #{ generate_trajectory_steps()

    def generate_trajectory_steps(self, waypoints, step_size):
        """
        Generate interpolated trajectory points between waypoints.

        Args:
            waypoints (list): List of [x, y, z] waypoints.
            step_size (float): Distance between consecutive trajectory points.

        Returns:
            list: A list of [x, y, z] points including intermediates.
        """
        trajectory = []

        for i in range(len(waypoints) - 1):
            start = np.array(waypoints[i], dtype=float)
            end = np.array(waypoints[i + 1], dtype=float)

            # Compute segment length
            segment_vec = end - start
            segment_dist = np.linalg.norm(segment_vec)

            # Number of interpolation steps
            if segment_dist == 0:
                continue
            num_steps = int(np.floor(segment_dist / step_size))

            # Unit direction vector
            direction = segment_vec / segment_dist

            # Generate intermediate points
            for j in range(num_steps):
                point = start + direction * step_size * j
                trajectory.append(point.tolist())

        # Append final waypoint
        trajectory.append(waypoints[-1])

        return trajectory

    # #} end of generate_trajectory_steps()

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
