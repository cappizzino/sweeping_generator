#!/usr/bin/python3

import rospy
import numpy
import tf2_ros
import tf2_geometry_msgs
import math
import utm
from mrs_msgs.msg import ControlManagerDiagnostics,Reference, ReferenceStamped
from mrs_modules_msgs.msg import OctomapPlannerDiagnostics
from mrs_msgs.srv import PathSrv,PathSrvRequest, ReferenceStampedSrv, ReferenceStampedSrvRequest
from mrs_msgs.srv import TransformReferenceSrv, TransformReferenceSrvRequest
from mrs_msgs.srv import Vec1,Vec1Response
from mrs_msgs.srv import String, StringRequest
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import UInt8
from std_srvs.srv import Trigger, TriggerResponse
from rospy import ServiceException
from waypoints_latlog import SwarmInputs, compute_swarm_waypoint_lines

class Node:

    # #{ __init__(self)

    def __init__(self):

        rospy.init_node("sweeping_generator", anonymous=True)
        self.is_initialized = False

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
        self.timer_comm_rate = rospy.get_param("~timer_comm/rate")
        self.timeout_comm = rospy.get_param("~timer_comm/timeout")

        self.is_comm_initialized = False
        self.leader_lost = False
        self.timeout_counter = 0
        self.timeout_limit = int(self.timeout_comm / (1.0 / self.timer_comm_rate))

        self.angle = rospy.get_param("~angle")
        self.lateral_scale = rospy.get_param("~lateral_scale")
        self.side = rospy.get_param("~side")

        self.octomap_planner_set = rospy.get_param("~octomap_planner_set", True)
        self.leader_reference_set = rospy.get_param("~leader_reference_set", False)
        self.heading_leader_zero = rospy.get_param("~heading_leader_zero", False)

        self.emergency_hover_z = rospy.get_param("~emergency_hover_z", 2.0)

        if self.id != 0:
            if self.id % 2 == 0:
                self.offset_sign = math.ceil(self.id/2)
            else:
                self.offset_sign = -math.ceil(self.id/2)
        self.r = self.side / numpy.sqrt(3.0)
        self.wing_index = math.ceil(self.id/2)

        self.follower_path = rospy.get_param("~follower_path", "line")

        self.has_goal = False

        # self.waypoint_list = rospy.get_param('~waypoint_list', [])
        self.waypoint_list = []

        self.offset_waypoint_utm_flag = rospy.get_param('~offset_waypoint_utm_flag', True)
        self.automated_calculation = rospy.get_param('~automated_calculation', False)

        if self.automated_calculation:
            inp = SwarmInputs(
                lat_start=rospy.get_param("~lat_start", 41.2209406896861),
                lon_start=rospy.get_param("~lon_start", -8.527200278531424),
                lat_end=rospy.get_param("~lat_end", 41.22115148976369),
                lon_end=rospy.get_param("~lon_end", -8.527152346726584),
                drones=self.drone_list,
                distance_between_drones=rospy.get_param("~distance_between_drones", 3.0),
                distance_line=rospy.get_param("~distance_line", 10.0),
                altitude=rospy.get_param("~altitude", 5.0)
            )
            lines = compute_swarm_waypoint_lines(inp)

            # Output format 1: (num_lines) x (num_drones)
            print("Lines (line_index -> list of (lat,lon,alt) per drone in input order):")
            for j, line in enumerate(lines):
                print(f"  line {j}: {line[self.id]}")
                self.waypoint_list.append(line[self.id])  # add waypoint of this drone to the list
        else:
            self.waypoint_list = rospy.get_param('~waypoint_list', [])

        self.offset_waypoint_utm = []
        self.waypoint_count = 0
        self.publish_leader_index = False
        # self.waypoint_frame = rospy.get_param('~waypoint_frame', "fixed_origin")
        self.leader_index = None
        self.local_index = None
        self.autonomous_fallback = False
        self.last_sent_index = None

        if self.leader_swarm == False:
            waypoint_utm = []
            if self.octomap_planner_set:
                for waypoint in self.waypoint_list:
                    wp = self.latlon_to_utm(waypoint[0], waypoint[1], waypoint[2])
                    waypoint_utm.append(wp)
                self.offset_waypoint_utm = self.offset_utm_path(waypoint_utm, -self.offset_sign * self.side)

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(60.0))
        self.listener = tf2_ros.TransformListener(self.tf_buffer)

        self.main_count_max = rospy.get_param('~main_count_max', 10)
        self.main_count = 0
        self.fallback_goal_latch_timeout = rospy.get_param('~fallback_goal_latch_timeout', float(self.main_count_max))
        self.fallback_goal_completion_timeout = rospy.get_param('~fallback_goal_completion_timeout', 10.0)
        self.rejoin_policy = rospy.get_param('~rejoin_policy', 'keep_fallback')
        if self.rejoin_policy not in ['return', 'keep_fallback', 'wait_sync']:
            rospy.logwarn("[SweepingGenerator]: invalid ~rejoin_policy='%s', using 'keep_fallback'", self.rejoin_policy)
            self.rejoin_policy = 'keep_fallback'

        rospy.loginfo('[SweepingGenerator]: initialized')

        ## | ----------------------- subscribers ---------------------- |
        if self.leader_swarm:
            # self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)
            # self.sub_odom = rospy.Subscriber(f"/{self.uav_name}/estimation_manager/odom_main", Odometry, self.callbackOdom)
            self.sub_mpc = rospy.Subscriber(f"/{self.uav_name}/control_manager/mpc_tracker/predicted_trajectory_debugging", PoseArray, self.callbackMPC)
        else:
            if self.leader_reference_set == True:
                self.sub_reference = rospy.Subscriber(f"/{self.leader_name}/leader_reference", Reference, self.reference_cb)
            else:
                self.sub_index = rospy.Subscriber(f"/{self.leader_name}/leader_index", UInt8, self.index_cb)
        self.sub_odom = rospy.Subscriber(f"/{self.uav_name}/estimation_manager/odom_main", Odometry, self.callbackOdom)
        self.sub_control_manager_diag = rospy.Subscriber("~control_manager_diag_in", ControlManagerDiagnostics, self.callbackControlManagerDiagnostics)
        self.sub_octomap_planner_diag = rospy.Subscriber("~octomap_planner_diag_in", OctomapPlannerDiagnostics, self.callbackOctomapPlannerDiagnostics)

        ## | ----------------------- publishers ---------------------- |
        if self.leader_swarm:
            self.ref_pub = rospy.Publisher(f"/{self.uav_name}/leader_reference", Reference, queue_size=1)
            self.index_pub = rospy.Publisher(f"/{self.uav_name}/leader_index", UInt8, queue_size=1)
        else:
            self.ref_pub = rospy.Publisher(f"/{self.uav_name}/control_manager/reference", ReferenceStamped, queue_size=1)

        ## | --------------------- service servers -------------------- |
        if self.leader_swarm:
            self.ss_start = rospy.Service('~start_in', Vec1, self.callbackStart)
        self.ss_emergency = rospy.Service('~emergency_in', Trigger, self.callbackEmergency)

        ## | --------------------- service clients -------------------- |
        if self.leader_swarm:
            self.sc_path = rospy.ServiceProxy('~path_out', PathSrv)
        self.sc_octomap_planner = rospy.ServiceProxy(f'/{self.uav_name}/octomap_planner/reference', ReferenceStampedSrv)
        self.sc_octomap_planner_stop = rospy.ServiceProxy(f'/{self.uav_name}/octomap_planner/stop', Trigger)

        service_name = f'/{self.uav_name}/control_manager/transform_reference'
        rospy.loginfo('[SweepingGenerator]: waiting for service: {}'.format(service_name))
        rospy.wait_for_service(service_name)
        rospy.loginfo('[SweepingGenerator]: service available: {}'.format(service_name))
        self.sc_transform = rospy.ServiceProxy(service_name, TransformReferenceSrv)
        self.sc_landing = rospy.ServiceProxy(f'/{self.uav_name}/uav_manager/land', Trigger)

        # # Change estimator
        # service_name = f'/{self.uav_name}/estimation_manager/change_estimator'
        # self.sc_change_estimator = rospy.ServiceProxy(service_name, String)

        # rospy.loginfo('[SweepingGenerator]: all service clients initialized')
        # rospy.loginfo('[SweepingGenerator]: sleeping for 1 second before changing estimator')
        # rospy.sleep(3.0)
        # req = StringRequest()
        # req.value = "liosam"
        # try:
        #     result = self.sc_change_estimator(req)
        #     rospy.loginfo(f"Estimator change result: success={result.success}, message='{result.message}'")
        # except rospy.ServiceException as e:
        #     rospy.logerr(f"Service call failed: {e}")

        ## | ------------------------- timers ------------------------- |
        if self.leader_swarm:
            self.timer_main = rospy.Timer(rospy.Duration(1.0/self.timer_main_rate), self.timerMain)
            self.timer_pose = rospy.Timer(rospy.Duration(1.0/self.timer_pose_rate), self.timerPose)
            self.timer_index = rospy.Timer(rospy.Duration(1.0/self.timer_pose_rate), self.timerIndex)
        else:
            self.timer_comm = rospy.Timer(rospy.Duration(1.0/self.timer_comm_rate), self.timerComm)

        ## | -------------------- spin till the end ------------------- |

        self.is_initialized = True

        if not self.leader_swarm and self.octomap_planner_set and self.leader_reference_set == False:
            self.main_follower()

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

    # #{ callbackOctomapPlannerDiagnostics():

    def callbackOctomapPlannerDiagnostics(self, msg):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: getting OctomapPlanner diagnostics')

        self.sub_octomap_planner_diag = msg

    # #} end of callbackOctomapPlannerDiagnostics

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

        # if not self.is_initialized:
        #     return Vec1Response(False, "not initialized")

        if self.octomap_planner_set:

            if len(self.waypoint_list) == 0:
                rospy.logwarn('[SweepingGenerator]: no waypoints configured')
                return Vec1Response(False, "no waypoints configured")

            self.waypoint_count = 0
            self.publish_leader_index = False
            planner_response = None

            for waypoint in self.waypoint_list:
                rospy.logwarn('[SweepingGenerator]: processing waypoint index: {}'.format(self.waypoint_count))
                rospy.loginfo('[SweepingGenerator]: waypoint coordinates: x: {}, y: {}, z: {}'.format(waypoint[0], waypoint[1], waypoint[2]))

                ref = ReferenceStamped()
                ref.header.frame_id = "latlon_origin"
                ref.reference.position.x = waypoint[0]
                ref.reference.position.y = waypoint[1]
                ref.reference.position.z = 0.0
                ref.reference.heading = 0.0

                request = TransformReferenceSrvRequest()
                request.frame_id = self.uav_name + "/" + "liosam_origin"
                request.reference = ref

                try:
                    transform_response = self.sc_transform(request)
                    rospy.loginfo(f"Response: success={transform_response.success}, message='{transform_response.message}'")
                except rospy.ServiceException as e:
                    rospy.logerr(f"Service call failed: {e}")
                    return Vec1Response(False, "transform reference failed")

                point = ReferenceStampedSrvRequest()
                point.header.stamp = rospy.Time.now()
                point.header.frame_id = self.uav_name + "/" + "liosam_origin"
                point.reference.position.x = transform_response.reference.reference.position.x
                point.reference.position.y = transform_response.reference.reference.position.y
                point.reference.position.z = waypoint[2]
                point.reference.heading = 0.0

                rospy.loginfo('[SweepingGenerator]: sending point to octomap planner: x: {}, y: {}, z: {}'.format(point.reference.position.x, point.reference.position.y, point.reference.position.z))

                try:
                    self.has_goal = False
                    planner_response = self.sc_octomap_planner.call(point)
                    rospy.loginfo(f"Response: success={planner_response.success}, message='{planner_response.message}'")
                except rospy.ServiceException as e:
                    rospy.logerr(f"[SweepingGenerator]: octomap planner service not callable: {e}")
                    return Vec1Response(False, "octomap planner service not callable")

                if not planner_response.success:
                    rospy.logerr('[SweepingGenerator]: octomap planner setting failed, message: {}'.format(planner_response.message))
                    return Vec1Response(False, "octomap planner setting failed")

                # Wait for tracker to latch the goal before the next synchronization step.
                wait_timeout_s = float(self.main_count_max)
                deadline = rospy.Time.now() + rospy.Duration(wait_timeout_s)
                while not rospy.is_shutdown() and not self.has_goal and rospy.Time.now() < deadline:
                    rospy.sleep(0.1)

                if not self.has_goal:
                    rospy.logerr('[SweepingGenerator]: tracker goal not confirmed within {:.1f}s, continuing'.format(wait_timeout_s))

                if self.waypoint_count == 0:
                    rospy.loginfo('[SweepingGenerator]: waiting for first waypoint completion (self.has_goal == False)')
                    while not rospy.is_shutdown() and self.has_goal:
                        rospy.sleep(0.1)
                    self.publish_leader_index = True
                    rospy.logwarn('[SweepingGenerator]: first waypoint set. Press Enter to continue with remaining waypoints.')
                    try:
                        input('[SweepingGenerator] Press Enter to continue...')
                    except EOFError:
                        rospy.logwarn('[SweepingGenerator]: stdin unavailable, continuing automatically.')
                else:
                    rospy.loginfo('[SweepingGenerator]: waiting for waypoint {} completion (self.has_goal == False)'.format(self.waypoint_count))
                    while not rospy.is_shutdown() and self.has_goal:
                        rospy.sleep(0.1)
                    rospy.loginfo('[SweepingGenerator]: waypoint {} completed, moving to next'.format(self.waypoint_count))

                if self.waypoint_count < (len(self.waypoint_list) - 1):
                    self.waypoint_count += 1

            self.publish_leader_index = False

            if planner_response and planner_response.success:
                rospy.loginfo('[SweepingGenerator]: octomap planner set')
                return Vec1Response(True, "starting")

            rospy.loginfo('[SweepingGenerator]: octomap planner setting failed')
            return Vec1Response(False, "octomap planner setting failed")

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

    # #{ callbackEmergency():

    def callbackEmergency(self, req):
        rospy.logwarn('[SweepingGenerator]: Emergency stop triggered')

        # Optional: prevent duplicate execution
        if getattr(self, 'emergency_active', False):
            rospy.logwarn('[SweepingGenerator]: Emergency already active')
            return TriggerResponse(success=True, message='emergency already active')

        self.emergency_active = True

        failures = []

        # 1) Stop octomap planner
        try:
            stop_resp = self.sc_octomap_planner_stop()
            if stop_resp.success:
                rospy.loginfo('[SweepingGenerator]: Octomap planner stopped successfully')
            else:
                msg = f'planner stop failed: {stop_resp.message}'
                rospy.logerr(f'[SweepingGenerator]: {msg}')
                failures.append(msg)
        except ServiceException as e:
            msg = f'planner stop service call failed: {e}'
            rospy.logerr(f'[SweepingGenerator]: {msg}')
            failures.append(msg)

        # 2) Clear planner path / reset goal
        try:
            clear_req = ReferenceStampedSrvRequest()
            clear_req.header.stamp = rospy.Time.now()
            clear_req.header.frame_id = f'{self.uav_name}/local_origin'
            clear_req.reference.position.x = 0.0
            clear_req.reference.position.y = 0.0
            clear_req.reference.position.z = self.emergency_hover_z  # define this explicitly
            clear_req.reference.heading = 0.0

            clear_resp = self.sc_octomap_planner(clear_req)
            if clear_resp.success:
                rospy.loginfo('[SweepingGenerator]: Octomap planner path cleared successfully')
            else:
                msg = f'planner clearing failed: {clear_resp.message}'
                rospy.logerr(f'[SweepingGenerator]: {msg}')
                failures.append(msg)
        except ServiceException as e:
            msg = f'planner clearing service call failed: {e}'
            rospy.logerr(f'[SweepingGenerator]: {msg}')
            failures.append(msg)
        except AttributeError as e:
            msg = f'invalid emergency altitude/configuration: {e}'
            rospy.logerr(f'[SweepingGenerator]: {msg}')
            failures.append(msg)

        # 3) Attempt landing regardless of planner result
        try:
            land_resp = self.sc_landing()
            if land_resp.success:
                rospy.loginfo('[SweepingGenerator]: Landing initiated successfully')
            else:
                msg = f'landing initiation failed: {land_resp.message}'
                rospy.logerr(f'[SweepingGenerator]: {msg}')
                failures.append(msg)
        except ServiceException as e:
            msg = f'landing service call failed: {e}'
            rospy.logerr(f'[SweepingGenerator]: {msg}')
            failures.append(msg)

        if failures:
            return TriggerResponse(
                success=False,
                message='emergency executed with errors: ' + '; '.join(failures)
            )

        return TriggerResponse(success=True, message='emergency stop executed successfully')

    # #} end of callbackEmergency

    # #{ reference_cb():

    def reference_cb(self, msg):

        # communication initialized
        self.is_comm_initialized = True

        # Reset timeout counter
        self.timeout_counter = 0

        # Extract leader state
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # Extract leader heading (yaw)
        heading = msg.heading

        if self.follower_path == "v_formation":
            # Compute follower offset in leader frame
            dx_body = -self.r * self.wing_index
            dy_body = self.offset_sign * (self.side / 2.0)

            # Rotate offset to world frame
            dx_world = dx_body * numpy.cos(heading) - dy_body * numpy.sin(heading)
            dy_world = dx_body * numpy.sin(heading) + dy_body * numpy.cos(heading)

        else:
            # Compute follower offset in leader frame
            dx_body = 0.0
            dy_body = self.offset_sign * self.side

            # Rotate offset to world frame
            dx_world = dx_body * numpy.cos(heading) - dy_body * numpy.sin(heading)
            dy_world = dx_body * numpy.sin(heading) + dy_body * numpy.cos(heading)

        # Compute follower desired position
        target_x = x + dx_world
        target_y = y + dy_world
        target_z = z  # same altitude

        if self.octomap_planner_set:
            point = ReferenceStampedSrvRequest()
            point.header.stamp = rospy.Time.now()
            point.header.frame_id = self.uav_name + "/" + self.frame_publish
            point.reference.position.x = target_x
            point.reference.position.y = target_y
            point.reference.position.z = target_z
            point.reference.heading = heading

            try:
                response = self.sc_octomap_planner.call(point)
            except:
                rospy.logerr('[SweepingGenerator]: octomap planner service not callable')
                pass

            if response.success:
                rospy.loginfo('[SweepingGenerator]: octomap planner set for follower')
            else:
                rospy.loginfo('[SweepingGenerator]: octomap planner setting failed for follower, message: {}'.format(response.message))
        else:
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
    
    # #} end of reference_cb()

    # #{ index_cb():

    def index_cb(self, msg):

        # communication initialized
        self.is_comm_initialized = True

        # Leader lost flag reset
        self.leader_lost = False

        # Reset timeout counter
        self.timeout_counter = 0

        # Index of the leader waypoint
        self.leader_index = int(msg.data)
        if self.rejoin_policy == 'return':
            self.autonomous_fallback = False
            self.local_index = None

        rospy.loginfo_once('[SweepingGenerator]: getting leader index message: {}'.format(self.leader_index))

    ## | ------------------------- timers ------------------------- |

    # #{ timerMain()

    def timerMain(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: main timer spinning')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):
            if self.sub_control_manager_diag.tracker_status.have_goal:
                self.has_goal = True
                # rospy.loginfo('[SweepingGenerator]: tracker has goal')
            else:
                self.has_goal = False
                # rospy.loginfo('[SweepingGenerator]: waiting for command')

        if isinstance(self.sub_octomap_planner_diag, OctomapPlannerDiagnostics):
            if self.sub_octomap_planner_diag.idle:
                self.idle = True
                # rospy.loginfo('[SweepingGenerator]: Octomap planner is idle.')
            else:
                self.idle = False
                # rospy.loginfo('[SweepingGenerator]: Octomap planner is not idle.')
        
        self.main_count += 1
        # rospy.loginfo('[SweepingGenerator]: Flag: {}'.format(self.main_count))

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
                #rospy.loginfo(f"[SweepingGenerator]: using MPC pose index {i_mpc} out of {len(self.sub_mpc.poses)}")
                input_pose.pose = self.sub_mpc.poses[i_mpc]
                #rospy.loginfo(f"[SweepingGenerator]: leader at ({input_pose.pose.position.x:.2f}, {input_pose.pose.position.y:.2f}, {input_pose.pose.position.z:.2f})")
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
                    if self.waypoint_count != 0:
                        self.ref_pub.publish(leader_point)
                else:
                    rospy.logwarn("Transformation failed.")
        
        else:
            rospy.logwarn("No messages from leader.")

    # #} end of timerPose()

    # #{ timerIndex()

    def timerIndex(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: index timer spinning')

        if not self.publish_leader_index:
            return
        self.index_pub.publish(self.waypoint_count)

    # #} end of timerIndex()

    # #{ timerComm()

    def timerComm(self, event=None):

        if not self.is_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: comm timer spinning')

        if not self.is_comm_initialized:
            return

        rospy.loginfo_once('[SweepingGenerator]: communication with leader established')

        if isinstance(self.sub_control_manager_diag, ControlManagerDiagnostics):
            if self.sub_control_manager_diag.tracker_status.have_goal:
                self.has_goal = True
            else:
                self.has_goal = False

        rospy.loginfo('[SweepingGenerator]: self.timeout_counter: {}'.format(self.timeout_counter))

        if self.timeout_counter >= self.timeout_limit and not self.leader_lost:
            rospy.logwarn('[SweepingGenerator]: no messages from leader detected yet')
            self.leader_lost = True
        
        if not self.leader_lost:
            self.timeout_counter += 1

    # #} end of timerComm()

    def main_follower(self):

        if not self.is_initialized:
            return

        while not rospy.is_shutdown():
            waypoint_count_total = len(self.offset_waypoint_utm) if self.offset_waypoint_utm_flag else len(self.waypoint_list)
            if waypoint_count_total == 0:
                rospy.logwarn_throttle(2.0, '[SweepingGenerator]: no waypoints available for follower')
                rospy.sleep(0.2)
                continue

            if self.leader_lost and not self.autonomous_fallback:
                if self.leader_index is not None:
                    self.local_index = self.leader_index
                elif self.local_index is None:
                    self.local_index = 0
                self.local_index = max(0, min(self.local_index, waypoint_count_total - 1))
                self.autonomous_fallback = True
                self.last_sent_index = None
                rospy.logwarn('[SweepingGenerator]: leader lost, autonomous fallback from index {}'.format(self.local_index))

            if self.autonomous_fallback:
                if self.local_index is None:
                    self.local_index = 0

                if self.rejoin_policy == 'keep_fallback':
                    target_index = self.local_index
                elif self.rejoin_policy == 'wait_sync':
                    if (not self.leader_lost) and (self.leader_index is not None):
                        if self.leader_index >= self.local_index:
                            rospy.logwarn('[SweepingGenerator]: leader caught up (leader=%d, local=%d), resuming leader mode', self.leader_index, self.local_index)
                            self.autonomous_fallback = False
                            self.local_index = None
                            target_index = self.leader_index
                        else:
                            rospy.loginfo_throttle(2.0, '[SweepingGenerator]: wait_sync, waiting leader to catch up (leader=%d, local=%d)', self.leader_index, self.local_index)
                            rospy.sleep(0.1)
                            continue
                    else:
                        target_index = self.local_index
                else:
                    if (not self.leader_lost) and (self.leader_index is not None):
                        self.autonomous_fallback = False
                        self.local_index = None
                        target_index = self.leader_index
                    else:
                        target_index = self.local_index
            else:
                if self.leader_index is None:
                    rospy.sleep(0.1)
                    continue
                target_index = self.leader_index

            if target_index < 0:
                rospy.logwarn_throttle(2.0, '[SweepingGenerator]: received negative index {}'.format(target_index))
                rospy.sleep(0.1)
                continue

            if target_index >= waypoint_count_total:
                if self.autonomous_fallback:
                    self.local_index = waypoint_count_total - 1
                    rospy.loginfo_throttle(2.0, '[SweepingGenerator]: fallback reached last waypoint, holding position')
                else:
                    rospy.loginfo_throttle(2.0, '[SweepingGenerator]: leader index {} out of range [0, {}], waiting...'.format(target_index, waypoint_count_total - 1))
                rospy.sleep(0.1)
                continue

            if self.last_sent_index != target_index:
                if self.offset_waypoint_utm_flag:
                    point = ReferenceStampedSrvRequest()
                    point.header.stamp = rospy.Time.now()
                    point.header.frame_id = self.uav_name + "/" + self.frame_publish
                    point.reference.position.x = self.offset_waypoint_utm[target_index][0]
                    point.reference.position.y = self.offset_waypoint_utm[target_index][1]
                    point.reference.position.z = self.offset_waypoint_utm[target_index][2]
                    point.reference.heading = 0.0
                else:
                    waypoint = self.waypoint_list[target_index]
                    rospy.loginfo('[SweepingGenerator]: waypoint coordinates: x: {}, y: {}, z: {}'.format(waypoint[0], waypoint[1], waypoint[2]))

                    ref = ReferenceStamped()
                    ref.header.frame_id = "latlon_origin"
                    ref.reference.position.x = waypoint[0]
                    ref.reference.position.y = waypoint[1]
                    ref.reference.position.z = 0.0
                    ref.reference.heading = 0.0

                    request = TransformReferenceSrvRequest()
                    request.frame_id = self.uav_name + "/" + "liosam_origin"
                    request.reference = ref

                    try:
                        transform_response = self.sc_transform(request)
                        rospy.loginfo(f"Response: success={transform_response.success}, message='{transform_response.message}'")
                    except rospy.ServiceException as e:
                        rospy.logerr(f"Service call failed: {e}")
                        rospy.sleep(0.2)
                        continue

                    point = ReferenceStampedSrvRequest()
                    point.header.stamp = rospy.Time.now()
                    point.header.frame_id = self.uav_name + "/" + "liosam_origin"
                    point.reference.position.x = transform_response.reference.reference.position.x
                    point.reference.position.y = transform_response.reference.reference.position.y
                    point.reference.position.z = waypoint[2]
                    point.reference.heading = 0.0

                    rospy.loginfo('[SweepingGenerator]: sending point to octomap planner: x: {}, y: {}, z: {}'.format(point.reference.position.x, point.reference.position.y, point.reference.position.z))

                try:
                    self.has_goal = False
                    response = self.sc_octomap_planner.call(point)
                    rospy.loginfo(f"Response: success={response.success}, message='{response.message}'")
                except rospy.ServiceException as e:
                    rospy.logerr(f'[SweepingGenerator]: octomap planner service not callable: {e}')
                    rospy.sleep(0.2)
                    continue

                if not response.success:
                    rospy.logerr('[SweepingGenerator]: octomap planner setting failed, message: {}'.format(response.message))
                    rospy.sleep(0.2)
                    continue

                self.last_sent_index = target_index

                if self.autonomous_fallback:
                    latch_timeout_s = float(self.fallback_goal_latch_timeout)
                    deadline = rospy.Time.now() + rospy.Duration(latch_timeout_s)
                    while not rospy.is_shutdown() and not self.has_goal and rospy.Time.now() < deadline:
                        rospy.sleep(0.1)

                    if not self.has_goal:
                        rospy.logwarn('[SweepingGenerator]: fallback goal latch timeout ({:.1f}s) at index {}, moving on'.format(latch_timeout_s, target_index))
                        if self.local_index < waypoint_count_total:
                            self.local_index += 1
                        continue

                    completion_timeout_s = float(self.fallback_goal_completion_timeout)
                    completion_deadline = rospy.Time.now() + rospy.Duration(completion_timeout_s)
                    while not rospy.is_shutdown() and self.has_goal and rospy.Time.now() < completion_deadline:
                        rospy.sleep(0.1)

                    if self.has_goal:
                        rospy.logwarn('[SweepingGenerator]: fallback completion timeout ({:.1f}s) at index {}, giving up and continuing'.format(completion_timeout_s, target_index))
                        if target_index >= (waypoint_count_total - 1):
                            # Mark fallback as done when the last waypoint never clears have_goal.
                            self.local_index = waypoint_count_total
                        else:
                            self.local_index = target_index + 1
                        continue

                    if self.local_index < waypoint_count_total:
                        self.local_index += 1
                    continue

            while not rospy.is_shutdown() and (not self.autonomous_fallback) and (not self.leader_lost) and self.leader_index == target_index:
                rospy.sleep(0.1)

    def transform_pose(self, input_pose, target_frame):
        """Transforms a PoseStamped to the target_frame using TF2."""
        try:

            # Wait for the transform to become available
            self.tf_buffer.can_transform(target_frame, 
                                        input_pose.header.frame_id, 
                                        rospy.Time(0),
                                        rospy.Duration(5.0))
            
            # Perform the transformation
            output_pose = self.tf_buffer.transform(input_pose, target_frame, rospy.Duration(5.0))
            return output_pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr(f"Transform failed: {e}")
            return None

    def rotate(self, x, y, theta):
        return (
            x * numpy.cos(theta) - y * numpy.sin(theta),
            x * numpy.sin(theta) + y * numpy.cos(theta)
        )

    def latlon_to_utm(self, lat, lon, alt=0.0):
        """Convert latitude and longitude to UTM coordinates."""
        u = utm.from_latlon(lat, lon)
        return u[0], u[1], alt  # x, y, z

    def offset_utm_path(self, original_waypoint, side):
        """
        original_waypoint: list of waypoints
        side: float → offset distance in meters (+left, -right)

        returns: new list of offset UTM points
        """

        offset_points = []

        for i in range(len(original_waypoint)):
            # Get current point
            x0, y0 = original_waypoint[i][0], original_waypoint[i][1]

            # Determine direction vector
            if i < len(original_waypoint) - 1:
                x1, y1 = original_waypoint[i+1][0], original_waypoint[i+1][1]
            else:
                # Last point: use previous direction
                x1, y1 = original_waypoint[i-1][0], original_waypoint[i-1][1]

            # Direction vector
            dx = x1 - x0
            dy = y1 - y0
            length = math.sqrt(dx*dx + dy*dy)

            if length == 0:
                rospy.logwarn("Degenerate point detected.")
                offset_points.append((x0, y0, original_waypoint[i][2]))  # keep original z
                continue

            # Normalize direction
            dx /= length
            dy /= length

            # Perpendicular vector (rotate 90° CCW)
            nx = -dy
            ny = dx

            # Apply side offset
            ox = x0 + nx * side
            oy = y0 + ny * side

            rospy.loginfo(f"Original: ({x0}, {y0}), Offset: ({ox}, {oy}), Side: {side}")
            offset_points.append((ox, oy, original_waypoint[i][2]))  # keep original z

        return offset_points

if __name__ == '__main__':
    try:
        node = Node()
    except rospy.ROSInterruptException:
        pass
