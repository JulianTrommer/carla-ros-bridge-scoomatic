#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import rospy
import tf

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive


# ==============================================================================
# ---ScoomaticController--------------------------------------------------------
# ==============================================================================


class ScoomaticNavigation(object):

    def __init__(self):
        self.loop_rate = rospy.Rate(10)
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')

        # ==========================================
        # -- Subscriber ----------------------------
        # ==========================================

	# velocity commands from move_base
	self.cmd_vel = Twist()
	self.cmd_vel_subscriber = rospy.Subscriber(
	    "/cmd_vel", Twist, self.cmd_vel_updated)

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

        # ackermann drive message for the ackermann control
	self.twist_pub = rospy.Publisher("carla/{}/twist".format(self.role_name), Twist, queue_size=50)


    def cmd_vel_updated(self, cmd_vel):
	self.twist_pub.publish(cmd_vel)


    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.cmd_vel_subscriber.unregister()
	self.twist_pub.unregister()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("scoomatic_navigation", anonymous=True)

    scoomatic_navigation = None

    try:
        scoomatic_navigation = ScoomaticNavigation()
	while not rospy.is_shutdown():
	    pass
    finally:
        if scoomatic_navigation is not None:
            del scoomatic_navigation
        rospy.loginfo("Done")



if __name__ == '__main__':
    main()
