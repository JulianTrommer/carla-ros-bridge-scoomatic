#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import rospy
import math

from geometry_msgs.msg import Twist
from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import CarlaEgoVehicleStatus
# ==============================================================================
# ---ScoomaticController--------------------------------------------------------
# ==============================================================================


class ScoomaticController(object):

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

	self.vehicle_status = CarlaEgoVehicleStatus()
	self.vehicle_status_subscriber = rospy.Subscriber(
	    "carla/ego_vehicle/vehicle_status", CarlaEgoVehicleStatus, self.vehicle_status_updated)

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

        # ackermann drive message for the ackermann control
	self.ackermann_pub = rospy.Publisher("scoomatic/ackermann_cmd", AckermannDrive, queue_size=50)

	self.vehicle_status_pub = rospy.Publisher("scoomatic/vehicle_status", CarlaEgoVehicleStatus, queue_size=50)

	path = os.path.abspath(os.path.join(os.path.dirname( __file__ ), "..", "..", "config", "pid_params.yaml"))
	os.system("rosrun dynamic_reconfigure dynparam load /carla/ego_vehicle/ackermann_control {path}".format(path=path))


    def cmd_vel_updated(self, cmd_vel):
	self.ackermann_drive = AckermannDrive()

	self.ackermann_drive.speed = cmd_vel.linear.x
	if cmd_vel.linear.x == 0 or cmd_vel.angular.z == 0:	
	    self.ackermann_drive.steering_angle = 0
	else:
	    self.ackermann_drive.steering_angle = math.atan(1.0 / (cmd_vel.linear.x / cmd_vel.angular.z))

	self.ackermann_pub.publish(self.ackermann_drive)

    def vehicle_status_updated(self, vehicle_status):
	self.vehicle_status = vehicle_status
	if self.vehicle_status.control.reverse == True and self.vehicle_status.velocity > 0:
	    self.vehicle_status.velocity = -self.vehicle_status.velocity
	self.vehicle_status_pub.publish(self.vehicle_status)


    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.cmd_vel_subscriber.unregister()
	self.ackermann_pub.unregister()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("scoomatic_controller", anonymous=True)

    scoomatic_controller = None

    try:
        scoomatic_controller = ScoomaticController()
	while not rospy.is_shutdown():
	    pass
    finally:
        if scoomatic_controller is not None:
            del scoomatic_controller
        rospy.loginfo("Done")



if __name__ == '__main__':
    main()
