#!/usr/bin/python3

import rospy
import numpy as np
from mrs_msgs.msg import Reference, ReferenceStamped

class FollowerController:
    def __init__(self, uav_name, side, offset_sign):
        """
        uav_name: 'uav2' or 'uav3'
        side: side length of equilateral triangle (meters)
        offset_sign: -1 for left follower, +1 for right follower
        """
        self.uav_name = uav_name
        self.side = side
        self.offset_sign = offset_sign
        self.r = side / np.sqrt(3.0)

        # Publisher: position reference for MRS UAV
        self.ref_pub = rospy.Publisher(f"/{uav_name}/control_manager/reference", ReferenceStamped, queue_size=1)

        # Subscriber: leader reference
        rospy.Subscriber("/uav1/leader_reference", Reference, self.reference_cb)

    def reference_cb(self, msg):
        # Extract leader state
        x = msg.position.x
        y = msg.position.y
        z = msg.position.z

        # Extract leader heading (yaw)
        heading = msg.heading

        # Compute follower offset in leader frame
        dx_body = -self.r
        dy_body = self.offset_sign * (self.side / 2.0)

        # Rotate offset to world frame
        dx_world = dx_body * np.cos(heading) - dy_body * np.sin(heading)
        dy_world = dx_body * np.sin(heading) + dy_body * np.cos(heading)

        # Compute follower desired position
        target_x = x + dx_world
        target_y = y + dy_world
        target_z = z  # same altitude

        # Publish reference
        ref = ReferenceStamped()
        ref.header.stamp = rospy.Time.now()
        ref.header.frame_id = self.uav_name + "/utm_origin"
        ref.reference.position.x = target_x
        ref.reference.position.y = target_y
        ref.reference.position.z = target_z
        ref.reference.heading = heading

        self.ref_pub.publish(ref)

        # rospy.loginfo_throttle(1.0, f"[{self.uav_name}] following leader at ({target_x:.2f}, {target_y:.2f})")


if __name__ == "__main__":
    rospy.init_node("follower_controller")

    # Parameters
    side = rospy.get_param("~formation_side", 3.0)

    # Create two followers
    left_follower = FollowerController("uav2", side, offset_sign=-1)
    right_follower = FollowerController("uav3", side, offset_sign=+1)

    rospy.spin()
