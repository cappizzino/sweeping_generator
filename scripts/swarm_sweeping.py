#!/usr/bin/python3

import rospy
import numpy
import tf2_ros
import tf2_geometry_msgs
import math
from mrs_msgs.msg import ControlManagerDiagnostics,Reference, ReferenceStamped
from mrs_msgs.srv import PathSrv,PathSrvRequest
from mrs_msgs.srv import Vec1,Vec1Response
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("sweeping_generator", anonymous=True)

        ## | --------------------- load parameters -------------------- |
        self.uav_name = rospy.get_param("~uav_name", "uav1")
        self.drone_list = rospy.get_param('~uav_names', [])
        self.computer_name = rospy.get_param("~computer_name", "computer1")

        self.leader_name = self.drone_list[0]

        self.leader_swarm = False
        if self.uav_name == self.leader_name:
            self.leader_swarm = True
        
        self.id = 0
        for name in self.drone_list:
            if name == self.uav_name: break
            self.id += 1
        rospy.loginfo("ID: %d", self.id)

        self.frame_id = rospy.get_param("~frame_id")
        self.frame_publish = rospy.get_param("~frame_publish")

        self.leader_msg = rospy.get_param("~leader_msg")

        self.center_x = rospy.get_param("~center/x")
        self.center_y = rospy.get_param("~center/y")
        self.center_z = rospy.get_param("~center/z")

        self.dimensions_x = rospy.get_param("~dimensions/x")
        self.dimensions_y = rospy.get_param("~dimensions/y")

        self.timer_main_rate = rospy.get_param("~timer_main/rate")
        self.timer_pose_rate = rospy.get_param("~timer_pose/rate")

        self.angle = rospy.get_param("~angle")
        self.lateral_scale = rospy.get_param("~lateral_scale")
        self.side = rospy.get_param("~side")

        if self.id != 0:
            if self.id % 2 == 0:
                self.offset_sign = math.ceil(self.id/2)
            else:
                self.offset_sign = -math.ceil(self.id/2)
        self.r = self.side / numpy.sqrt(3.0)
        self.wing_index = math.ceil(self.id/2)

        rospy.loginfo('[SweepingGenerator]: initialized')

        ## | ----------------------- subscribers ---------------------- |
        if self.leader_swarm:
            self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)
            self.sub_odom = rospy.Subscriber(f"/{self.uav_name}/estimation_manager/odom_main", Odometry, self.callbackOdom)
            self.sub_mpc = rospy.Subscriber(f"/{self.uav_name}/control_manager/mpc_tracker/predicted_trajectory_debugging", PoseArray, self.callbackMPC)
        else:
            self.sub_reference = rospy.Subscriber(f"/{self.leader_name}/leader_reference", Reference, self.reference_cb)

        ## | ----------------------- publishers ---------------------- |
        if self.leader_swarm:
            self.ref_pub = rospy.Publisher(f"/{self.uav_name}/leader_reference", Reference, queue_size=1)
        else:
            self.ref_pub = rospy.Publisher(f"/{self.uav_name}/control_manager/reference", ReferenceStamped, queue_size=1)

        ## | --------------------- service servers -------------------- |
        if self.leader_swarm:
            self.ss_start = rospy.Service('~start_in', Vec1, self.callbackStart)

        ## | --------------------- service clients -------------------- |
        if self.leader_swarm:
            self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)

        ## | ------------------------- timers ------------------------- |
        if self.leader_swarm:
            self.timer_main = rospy.Timer(rospy.Duration(1.0/self.timer_main_rate), self.timerMain)
            self.timer_pose = rospy.Timer(rospy.Duration(1.0/self.timer_pose_rate), self.timerPose)

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        rospy.spin()

    # #} end of __init__()

    ## | ------------------------- methods ------------------------ |

    # #{ planPath()

    def planPath(self, step_size):

        rospy.loginfo('[SweepingGenerator]: planning path')

        # https://ctu-mrs.github.io/mrs_msgs/srv/PathSrv.html
        # -> https://ctu-mrs.github.io/mrs_msgs/msg/Path.html
        path_msg = PathSrvRequest()

        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()

        path_msg.path.fly_now = True

        path_msg.path.use_heading = True

        sign = 1.0

        # fill in the path with a sweeping pattern
        for i in numpy.arange(-self.dimensions_x/2.0, self.dimensions_x/2.0, step_size):

            for j in numpy.arange(-self.dimensions_y/2.0, self.dimensions_y/2.0, step_size):

                # https://ctu-mrs.github.io/mrs_msgs/msg/Reference.html
                point = Reference()

                point.position.x = self.center_x + i
                point.position.y = self.center_y + j*sign
                point.position.z = self.center_z
                point.heading = 0.0

                path_msg.path.points.append(point)

            if sign > 0.0:
                sign = -1.0
            else:
                sign = 1.0

        return path_msg

    def planInfinityPath(self, step_size, angle=0.0, lateral_scale=1.0):
        """
        Generate an infinity (Lemniscate of Bernoulli) path rotated by `angle` radians
        and stretched laterally by `lateral_scale`.

        Parameters:
        - step_size: sampling step in radians
        - angle: rotation in radians (counterclockwise)
        - lateral_scale: multiplier for lateral (y-axis) width of the infinity
        """
        rospy.loginfo('[SweepingGenerator]: planning infinity path (rotated + lateral scaled)')

        # Prepare path message
        path_msg = PathSrvRequest()
        path_msg.path.header.frame_id = self.frame_id
        path_msg.path.header.stamp = rospy.Time.now()
        path_msg.path.fly_now = True
        path_msg.path.use_heading = True

        # Infinity shape scale
        a = self.dimensions_x / 2.0  # horizontal scale
        scale_z = self.center_z      # keep altitude constant

        # Parametric range for infinity symbol
        t_values = numpy.arange(-numpy.pi, numpy.pi, step_size)

        prev_x, prev_y = None, None

        for t in t_values:
            point = Reference()

            # Lemniscate of Bernoulli (true infinity shape)
            denom = 1 + numpy.sin(t)**2
            x = (a * numpy.cos(t)) / denom
            y = (a * numpy.cos(t) * numpy.sin(t)) / denom

            # Apply lateral scaling
            y *= lateral_scale

            # Rotate by given angle
            x_rot = x * numpy.cos(angle) - y * numpy.sin(angle)
            y_rot = x * numpy.sin(angle) + y * numpy.cos(angle)

            # Center the rotated coordinates
            point.position.x = self.center_x + x_rot
            point.position.y = self.center_y + y_rot
            point.position.z = scale_z

            # Compute heading along the rotated path
            if prev_x is not None:
                dx = x_rot - prev_x
                dy = y_rot - prev_y
                point.heading = numpy.arctan2(dy, dx)
            else:
                point.heading = 0.0

            # Append point
            path_msg.path.points.append(point)

            prev_x, prev_y = x_rot, y_rot

        return path_msg


    # #} end of planPath()

    ## | ------------------------ callbacks ----------------------- |

    # #{ callbackControlManagerDiagnostics():

    def callbackControlManagerDiagnostics(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: getting ControlManager diagnostics')

        self.sub_control_manager_diag = msg

    # #} end of callbackControlManagerDiagnostics

    # #{ callbackOdom():

    def callbackOdom(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: getting Odometry message')

        self.sub_odom = msg

    # #} end of callbackOdom

    # #{ callbackMPC():

    def callbackMPC(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: getting MPC message')

        self.sub_mpc = msg

    # #} end of callbackMPC

    # #{ callbackStart():

    def callbackStart(self, req):

        if not self.is_initialized:
            return Vec1Response(False, "not initialized")

        # set the step size based on the service data
        step_size = req.goal

        path_msg = self.planInfinityPath(step_size, angle=self.angle, lateral_scale=self.lateral_scale)

        try:
            response = self.sc_path.call(path_msg)
        except:
            rospy.logerr('[SweepingGenerator]: path service not callable')
            pass

        if response.success:
            rospy.loginfo('[SweepingGenerator]: path set')
        else:
            rospy.loginfo('[SweepingGenerator]: path setting failed, message: {}'.format(response.message))

        return Vec1Response(True, "starting")

    # #} end of callbackStart

    # #{ reference_cb():

    def reference_cb(self, msg):
        # Extract leader state
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # Extract leader heading (yaw)
        heading = msg.heading

        # Compute follower offset in leader frame
        dx_body = -self.r * self.wing_index
        dy_body = self.offset_sign * (self.side / 2.0)

        # Rotate offset to world frame
        dx_world = dx_body * numpy.cos(heading) - dy_body * numpy.sin(heading)
        dy_world = dx_body * numpy.sin(heading) + dy_body * numpy.cos(heading)

        # Compute follower desired position
        target_x = x + dx_world
        target_y = y + dy_world
        target_z = z  # same altitude

        # Publish reference
        ref = ReferenceStamped()
        ref.header.stamp = rospy.Time.now()
        ref.header.frame_id = self.uav_name + "/" + self.frame_publish
        ref.reference.position.x = target_x
        ref.reference.position.y = target_y
        ref.reference.position.z = target_z
        ref.reference.heading = heading

        self.ref_pub.publish(ref)

        # rospy.loginfo_throttle(1.0, f"[{self.uav_name}] following leader at ({target_x:.2f}, {target_y:.2f})")
    
    # #} end of callbackStart

    ## | ------------------------- timers ------------------------- |

    # #{ timerMain()

    def timerMain(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: main timer spinning')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):
            if self.sub_control_manager_diag.tracker_status.have_goal:
                rospy.loginfo('[SweepingGenerator]: tracker has goal')
            else:
                rospy.loginfo('[SweepingGenerator]: waiting for command')

    # #} end of timerMain()

    # #{ timerPose()

    def timerPose(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: pose timer spinning')

        if (self.leader_msg == "odom"):

            if isinstance(self.sub_odom, Odometry):

                input_pose = PoseStamped()
                input_pose.header.stamp = rospy.Time(0)
                input_pose.header.frame_id = self.sub_odom.header.frame_id
                input_pose.pose = self.sub_odom.pose.pose
                target_frame = self.leader_name + "/" + self.frame_publish
                transformed_pose = self.transform_pose(input_pose, target_frame)

                if transformed_pose:
                    leader_point = Reference()
                    # Extract leader state
                    leader_point.position.x = transformed_pose.pose.position.x
                    leader_point.position.y = transformed_pose.pose.position.y
                    leader_point.position.z = transformed_pose.pose.position.z

                    # Extract leader heading (yaw)
                    q = transformed_pose.pose.orientation
                    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    leader_point.heading = numpy.arctan2(siny_cosp, cosy_cosp)

                    # rospy.loginfo(f"Leader at ({leader_point.position.x:.2f}, {leader_point.position.y:.2f}, {leader_point.position.z:.2f}, {leader_point.heading:.2f})")
                    self.ref_pub.publish(leader_point)
                else:
                    rospy.logwarn("Transformation failed.")
        
        elif (self.leader_msg == "mpc"):

            if isinstance(self.sub_mpc, PoseArray):

                input_pose = PoseStamped()
                input_pose.header.stamp = rospy.Time(0)
                input_pose.header.frame_id = self.sub_mpc.header.frame_id
                i_mpc = int(len(self.sub_mpc.poses)/4)
                input_pose.pose = self.sub_mpc.poses[i_mpc]
                target_frame = self.leader_name + "/" + self.frame_publish
                transformed_pose = self.transform_pose(input_pose, target_frame)

                if transformed_pose:
                    leader_point = Reference()
                    # Extract leader state
                    leader_point.position.x = transformed_pose.pose.position.x
                    leader_point.position.y = transformed_pose.pose.position.y
                    leader_point.position.z = transformed_pose.pose.position.z

                    # Extract leader heading (yaw)
                    q = transformed_pose.pose.orientation
                    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
                    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
                    leader_point.heading = numpy.arctan2(siny_cosp, cosy_cosp)

                    # rospy.loginfo(f"Leader at ({leader_point.position.x:.2f}, {leader_point.position.y:.2f}, {leader_point.position.z:.2f}, {leader_point.heading:.2f})")
                    self.ref_pub.publish(leader_point)
                else:
                    rospy.logwarn("Transformation failed.")
        
        else:
            rospy.logwarn("No messages from leader.")

    # #} end of timerMain()

    def transform_pose(self, input_pose, target_frame):
        """Transforms a PoseStamped to the target_frame using TF2."""
        try:
            # Initialize the TF2 buffer and listener
            tf_buffer = tf2_ros.Buffer()
            listener = tf2_ros.TransformListener(tf_buffer)

            # Wait for the transform to become available
            tf_buffer.can_transform(target_frame, 
                                    input_pose.header.frame_id, 
                                    rospy.Time(0), 
                                    rospy.Duration(3.0))
            
            # Perform the transformation
            output_pose = tf_buffer.transform(input_pose, target_frame)
            return output_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
            return None

    def rotate(self, x, y, theta):
        return (
            x * numpy.cos(theta) - y * numpy.sin(theta),
            x * numpy.sin(theta) + y * numpy.cos(theta)
        )

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
