#!/usr/bin/env python
#
# Copyright (c) 2019 Intel Corporation
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time

import os
import rospy
import carla

from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
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
	self.previous_scenario = 0

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

	# velocity of the walker (for dynamic obstacle visualization
	self.walker_vel = Twist()
	self.walker_vel_pub = rospy.Publisher("walker_vel", Twist, queue_size=50)


    def execute_scenario(self, scenario):
	if scenario.data == 0:
	    self.execute_previous_scenario()
	elif scenario.data == 1:
	    self.stop_scenario()
	    self.previous_scenario = 1
	    self.execute_scenario_one()
	elif scenario.data == 2:
	    self.stop_scenario()
	    self.previous_scenario = 2
	    self.execute_scenario_two()
	elif scenario.data == 3:
	    self.stop_scenario()
	    self.previous_scenario = 3
	    self.execute_scenario_three()

    def execute_previous_scenario(self):
	if self.previous_scenario > 0:
	    self.stop_scenario()
	    if self.previous_scenario == 1:
		self.execute_scenario_one()
	    elif self.previous_scenario == 2:
		self.execute_scenario_two()
	    elif self.previous_scenario == 3:
		self.execute_scenario_three()
	else:
	    rospy.logwarn("There was no scenario executed previously!")

    def execute_scenario_one(self):
	rospy.loginfo("Executing scenario 1...")
	self.set_scoomatic_pose(-18.85, 49.13, -1.4, 0.00, 0.00, 0.71, 0.70)
	self.spawn_walker(-19.25, -79.60, 1.3, 0.0, 90.0, 0.0, -0.025, 1.0, 0.0, 0.5)
	self.walker_vel.linear.x = -0.025 * 0.5
	self.walker_vel.linear.y = 0.5
	self.start_scenario(-19.25, 79.60, 1.3, 0.00, 0.00, 0.71, 0.70)

    def execute_scenario_two(self):
	rospy.loginfo("Executing scenario 2...")
	self.set_scoomatic_pose(-18.78, 66.92, -1.4, 0.00, 0.00, 1.0, 0.02)
	self.spawn_walker(-28.96, -75.45, 1.3, 0.0, 90.0, 0.0, -0.025, 1.0, 0.0, 0.25)
	self.walker_vel.linear.x = -0.025 * 0.25
	self.walker_vel.linear.y = 0.25
	self.start_scenario(-41.13, 68.14, -1.4, 0.00, 0.00, 1.00, 0.02)

    def execute_scenario_three(self):
	rospy.loginfo("Executing scenario 3...")
	self.set_scoomatic_pose(-16.78, 66.92, -1.4, 0.00, 0.00, 1.0, 0.02)
	self.spawn_walker(-28.96, -55.45, 1.3, 0.0, 270.0, 0.0, 0.025, -1.0, 0.0, 0.25)
	self.walker_vel.linear.x = 0.025 * 0.25
	self.walker_vel.linear.y = -0.25
	self.start_scenario(-41.13, 68.14, -1.4, 0.00, 0.00, 1.00, 0.02)


    def set_scoomatic_pose(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
	# set the pose of the scoomatic
	pose = PoseWithCovarianceStamped()
	pose.header.stamp = rospy.Time.now()
	pose.pose.pose.position.x = pos_x
	pose.pose.pose.position.y = pos_y
	pose.pose.pose.position.z = pos_z
	pose.pose.pose.orientation.x = orient_x
	pose.pose.pose.orientation.y = orient_y
	pose.pose.pose.orientation.z = orient_z
	pose.pose.pose.orientation.w = orient_w
	self.initialpose_pub.publish(pose)

    def spawn_walker(self, loc_x, loc_y, loc_z, pitch, yaw, roll, walk_x, walk_y, walk_z, walk_speed):
	# spawn the walker
	world = self.client.get_world()
	walker_bp = world.get_blueprint_library().filter("walker.pedestrian.*")[0]
	walker_bp.set_attribute('is_invincible', 'false')
	walker_sp = carla.Transform()
	walker_sp.location.x = loc_x
	walker_sp.location.y = loc_y
	walker_sp.location.z = loc_z
	walker_sp.rotation.pitch = pitch
	walker_sp.rotation.yaw = yaw
	walker_sp.rotation.roll = roll

	self.walker = world.spawn_actor(walker_bp, walker_sp)

	time.sleep(5)

	#spawn the walker control
	walker_control = carla.WalkerControl(carla.Vector3D(walk_x, walk_y, walk_z), walk_speed, False)
	self.walker.apply_control(walker_control)
	world.wait_for_tick()

    def start_scenario(self, pos_x, pos_y, pos_z, orient_x, orient_y, orient_z, orient_w):
	# start the scenario
	goal = PoseStamped()
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"	
	goal.pose.position.x = pos_x
	goal.pose.position.y = pos_y
	goal.pose.position.z = pos_z
	goal.pose.orientation.x = orient_x
	goal.pose.orientation.y = orient_y
	goal.pose.orientation.z = orient_z
	goal.pose.orientation.w = orient_w
	self.goal_pub.publish(goal)


    def stop_scenario(self):
	if self.walker is not None:
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
	    if scoomatic_scenario.walker is not None:
		scoomatic_scenario.walker_vel_pub.publish(scoomatic_scenario.walker_vel)
    finally:
        if scoomatic_scenario is not None:
            del scoomatic_scenario
        rospy.loginfo("Done")



if __name__ == '__main__':
    main()
