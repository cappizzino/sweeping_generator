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

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("swarm_unity", anonymous=True)

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
        if self.id != 0:
            if self.id % 2 == 0:
                self.offset_sign = math.ceil(self.id/2)
            else:
                self.offset_sign = -math.ceil(self.id/2)
        self.r = self.side / numpy.sqrt(3.0)
        self.wing_index = math.ceil(self.id/2)

        self.heading = 0.0
        self.humanPose = Point()
        self.humanPoseInitial = Point()
        self.getting_initial_human_pose = True

        ## | ----------------------- TF listener ---------------------- |
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        ## | ----------------------- subscribers ---------------------- |
        self.sub_unity = rospy.Subscriber(f"/uav/{self.leader_name}/path_0/pose_array", PoseArray, self.callbackUnity)

        ## | --------------------- Action clients -------------------- |
        self.action_name = f"/uav/{self.uav_name}/action/GoTo"
        self.goto_client = actionlib.SimpleActionClient(
            self.action_name, GoToAction
        )
        rospy.loginfo(f"[{self.uav_name}] Waiting for GoTo action server...")
        self.goto_client.wait_for_server(timeout=rospy.Duration(5.0))

        ## | ------------------------- timers ------------------------- |
        if self.leader_swarm:
            # Human Pose Timer
            self.timer_human = rospy.Timer(rospy.Duration(1.0/self.timer_human_rate), self.timerHuman)
        else:
            self.timer_point = rospy.Timer(rospy.Duration(1.0/self.timer_point_rate), self.timerPoint)
            self.timer_heading = rospy.Timer(rospy.Duration(1.0/self.timer_heading_rate), self.timerHeading)

        ## | -------------------- spin till the end ------------------- |
        rospy.loginfo('[Swarm_Unity]: initialized')
        self.is_initialized = True
        rospy.spin()

    # #} end of __init__()


    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackUnity():

    def callbackUnity(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[Swarm_Unity]: getting Unity message')

        self.sub_unity = msg

    # #} end of callbackUnity

    ## | ------------------- Action callbacks ------------------- |

    def goto_done_cb(self, state, result):
        rospy.loginfo(f"[{self.uav_name}] GoTo finished with result: {result.result}")

    def goto_active_cb(self):
        rospy.loginfo_once(f"[{self.uav_name}] GoTo goal is now active.")

    def goto_feedback_cb(self, feedback):
        rospy.loginfo_throttle(1.0, f"[{self.uav_name}] Feedback: {feedback.feedback:.2f}")

    ## | ------------------------- methods ------------------------ |

    # #{ reference_follower():

    def reference_follower(self, msg):

        goal_msg = GoToGoal()
        
        if self.leader_swarm:
            # Prepare goal message
            goal_msg.target = Point(msg.x, -msg.y, msg.z)
        else:
            # Extract leader state
            x = msg.x
            y = msg.y
            z = msg.z

            # Compute follower offset in leader frame
            dx_body = -self.r * self.wing_index
            dy_body = self.offset_sign * (self.side / 2.0)

            # Rotate offset to world frame
            dx_world = dx_body * numpy.cos(self.heading) - dy_body * numpy.sin(self.heading)
            dy_world = dx_body * numpy.sin(self.heading) + dy_body * numpy.cos(self.heading)

            # Compute follower desired position
            target_x = x + dx_world
            target_y = y + dy_world
            target_z = z  # same altitude

            # Prepare goal message
            goal_msg.target = Point(target_x, -target_y, target_z)

        # rospy.loginfo_throttle(1.0, f"[{self.uav_name}] Sending GoTo goal: ({target_x:.2f}, {target_y:.2f}, {target_z:.2f})")

        # Send goal asynchronously
        self.goto_client.send_goal(
            goal_msg,
            done_cb=self.goto_done_cb,
            active_cb=self.goto_active_cb,
            feedback_cb=self.goto_feedback_cb
        )
    
    # #} end of callbackStart

    # #{ compute_distance()

    def compute_distance(self, point: Point, reference: Point = None) -> float:
        """
        Compute the Euclidean distance between a given Point and a reference point (default: origin).

        Args:
            point (Point): Target position (e.g., human location)
            reference (Point, optional): Reference point (defaults to world origin)

        Returns:
            float: Euclidean distance
        """
        if reference is None:
            reference = Point(0.0, 0.0, 0.0)

        dx = point.x - reference.x
        dy = point.y - reference.y

        distance = math.sqrt(dx**2 + dy**2)
        # rospy.loginfo_throttle(1.0, f"Computed distance: {distance:.2f} m")

        return distance

    # #} end of compute_distance()


    ## | ------------------------- timers ------------------------- |

    # #{ timerHuman()
    def timerHuman(self, event=None):
        
        if not self.is_initialized:
            return

        rospy.loginfo_once('[Swarm_Unity]: human pose timer spinning')

        try:
            # Lookup human's transform relative to world
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, f"{self.human_name}", rospy.Time(0), rospy.Duration(0.5)
            )

            self.humanPose.x = transform.transform.translation.x
            self.humanPose.y = transform.transform.translation.y
            self.humanPose.z = transform.transform.translation.z

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(2.0, f"[{self.uav_name}] Could not get transform from {self.world_frame} to {self.human_name}")

        if self.getting_initial_human_pose:
            self.getting_initial_human_pose = False
            self.humanPoseInitial = transform.transform.translation
            rospy.loginfo(f"[{self.uav_name}] Initial human position recorded at ({self.humanPoseInitial.x:.2f}, {self.humanPoseInitial.y:.2f}, {self.humanPoseInitial.z:.2f})")

        else:
            distance = self.compute_distance(self.humanPose, self.humanPoseInitial)
            if distance > self.human_distance_threshold:
                rospy.loginfo(f"[{self.uav_name}] Human is within range, switching to human following mode.")
                self.reference_follower(self.humanPose)
                self.getting_initial_human_pose = True
    # #} end of timerHuman()

    # #{ timerPoint()

    def timerPoint(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[Swarm_Unity]: point timer spinning')

        if isinstance(self.sub_unity, PoseArray):

            input_pose = Point()
            if self.index == 0:
                i_mpc = 1
            else:
                i_mpc = int(len(self.sub_unity.poses)/self.index)
            input_pose.x = self.sub_unity.poses[i_mpc].position.x
            input_pose.y = self.sub_unity.poses[i_mpc].position.y
            input_pose.z = self.sub_unity.poses[i_mpc].position.z
            rospy.loginfo_throttle(1.0, f"[{self.uav_name}] Leader position: ({input_pose.x:.2f}, {input_pose.y:.2f}, {input_pose.z:.2f})")
            self.reference_follower(input_pose)

        else:
            rospy.logwarn("No messages from leader.")

    # #} end of timerPoint()

    # #{ timerHeading()

    def timerHeading(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[Swarm_Unity]: heading timer spinning')

        try:
            # Lookup leader's transform relative to world
            transform = self.tf_buffer.lookup_transform(
                self.world_frame, f"{self.leader_name}/base_link", rospy.Time(0), rospy.Duration(0.5)
            )

            # Extract quaternion rotation
            q = transform.transform.rotation
            quaternion = [q.x, q.y, q.z, q.w]

            # Convert quaternion to Euler angles
            (_, _, yaw) = euler_from_quaternion(quaternion)

            # Update and publish heading
            self.heading = yaw
            rospy.loginfo_throttle(1.0, f"[{self.uav_name}] Leader heading: {math.degrees(yaw):.2f}°")

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn_throttle(2.0, f"[{self.uav_name}] Could not get transform from {self.world_frame} to {self.leader_name}")

    # #} end of timerHeading()


if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
