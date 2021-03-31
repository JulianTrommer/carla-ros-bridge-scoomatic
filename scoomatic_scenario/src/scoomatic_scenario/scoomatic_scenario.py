#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import rospy
import carla

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from std_msgs.msg import UInt8

# ==============================================================================
# ---ScoomaticScenario----------------------------------------------------------
# ==============================================================================


class ScoomaticScenario(object):

    def __init__(self):
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')

	self.client = carla.Client(
	    rospy.get_param("~host", "localhost"), rospy.get_param("~port", "2000"))
	self.client.set_timeout(10.0)
	self.walker = None
	self.walker_controller = None

        # ==========================================
        # -- Subscriber ----------------------------
        # ==========================================

	# scenario message for selecting the executed scenario
	self.scenario = UInt8()
	self.scenario_subscriber = rospy.Subscriber(
	    "/scoomatic/scenario", UInt8, self.execute_scenario)

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

	# initial pose for the scoomatic
	self.initialpose_pub = rospy.Publisher("carla/ego_vehicle/initialpose", PoseWithCovarianceStamped, queue_size=50)

	# goal for the scoomatic
	self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=50)


    def execute_scenario(self, scenario):
	if scenario.data == 1:
	    self.execute_scenario_one()
	if scenario.data == 2:
	    self.stop_scenario()
	    self.execute_scenario_one()

    def execute_scenario_one(self):
	rospy.loginfo("Executing scenario 1...")
	# set the pose of the scoomatic
	self.pose = PoseWithCovarianceStamped()
	self.pose.pose.pose.position.x = -18.85
	self.pose.pose.pose.position.y = 49.13
	self.pose.pose.pose.position.z = -1.4
	self.pose.pose.pose.orientation.x = 0.00
	self.pose.pose.pose.orientation.y = 0.00
	self.pose.pose.pose.orientation.z = 0.71
	self.pose.pose.pose.orientation.w = 0.70
	self.initialpose_pub.publish(self.pose)
	# spawn the walker
	world = self.client.get_world()
	walker_bp = world.get_blueprint_library().filter("walker.pedestrian.*")[0]
	walker_bp.set_attribute('is_invincible', 'false')
	walker_sp = carla.Transform()
	walker_sp.location.x = -19.25
	walker_sp.location.y = -79.60
	walker_sp.location.z = 1.3
	walker_sp.rotation.pitch = 0.0
	walker_sp.rotation.yaw = 90.0
	walker_sp.rotation.roll = 0.0
	
	self.walker = world.spawn_actor(walker_bp, walker_sp)

	#spawn the walker control
	self.walker_control = carla.WalkerControl(carla.Vector3D(-0.025, 1.0, 0.0), 0.25, False)
	self.walker.apply_control(self.walker_control)
	world.wait_for_tick()

	# start the scenario
	self.goal = PoseStamped()
	self.goal.header.frame_id = "map"	
	self.goal.pose.position.x = -19.25
	self.goal.pose.position.y = 79.60
	self.goal.pose.position.z = 1.3
	self.goal.pose.orientation.x = 0.00
	self.goal.pose.orientation.y = 0.00
	self.goal.pose.orientation.z = 0.71
	self.goal.pose.orientation.w = 0.70
	self.goal_pub.publish(self.goal)

    def stop_scenario(self):
	self.walker.destroy()

    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.scenario_subscriber.unregister()
	self.stop_scenario()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("scoomatic_scenario", anonymous=True)

    scoomatic_scenario = None

    try:
        scoomatic_scenario = ScoomaticScenario()
	while not rospy.is_shutdown():
	    pass
    finally:
        if scoomatic_scenario is not None:
            del scoomatic_scenario
        rospy.loginfo("Done")



if __name__ == '__main__':
    main()
