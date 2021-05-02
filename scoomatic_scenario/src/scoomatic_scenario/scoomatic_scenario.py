#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import time

import rospy
import carla

from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped, Twist
from custom_msgs.msg import ScenarioParams

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

        # ==========================================
        # -- Subscriber ----------------------------
        # ==========================================

	# scenario message for selecting the executed scenario
	self.scenario = ScenarioParams()
	self.scenario_sub = rospy.Subscriber("/scoomatic/scenario", ScenarioParams, self.execute_scenario)

	# odometry of the scoomatic
	self.scoomatic_odometry = Odometry()
	self.odometry_sub = rospy.Subscriber("/odom", Odometry, self.odometry_received)

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

	# initial pose for the scoomatic
	self.initialpose_pub = rospy.Publisher("carla/ego_vehicle/initialpose", PoseWithCovarianceStamped, queue_size=50)

	# goal for the scoomatic
	self.goal_pub = rospy.Publisher("move_base_simple/goal", PoseStamped, queue_size=50)

	# velocity of the walker (for dynamic obstacle visualization)
	self.walker_vel = Twist()
	self.walker_vel_pub = rospy.Publisher("walker_vel", Twist, queue_size=50)

	# distance between the walker and the scoomatic
	self.distance_pub = rospy.Publisher("distance_to_walker", Float32, queue_size=50)


    def odometry_received(self, odometry):
	self.scoomatic_odometry = odometry

    def execute_scenario(self, scenario):
	self.stop_scenario()
	rospy.loginfo("Executing scenario...")
	self.set_scoomatic_pose(scenario.scoomatic_pose)
	self.spawn_walker(scenario.walker_params)	
	self.start_scenario(scenario.scoomatic_goal)

    def set_scoomatic_pose(self, scoomatic_pose):
	pose = PoseWithCovarianceStamped()
	pose.header.stamp = rospy.Time.now()
	pose.pose.pose.position.x = scoomatic_pose[0]
	pose.pose.pose.position.y = scoomatic_pose[1]
	pose.pose.pose.position.z = scoomatic_pose[2]
	pose.pose.pose.orientation.x = scoomatic_pose[3]
	pose.pose.pose.orientation.y = scoomatic_pose[4]
	pose.pose.pose.orientation.z = scoomatic_pose[5]
	pose.pose.pose.orientation.w = scoomatic_pose[6]
	self.initialpose_pub.publish(pose)

    def spawn_walker(self, walker_params):
	# spawn the walker
	world = self.client.get_world()
	walker_bp = world.get_blueprint_library().filter("walker.pedestrian.*")[0]
	walker_bp.set_attribute('is_invincible', 'false')
	walker_sp = carla.Transform()
	walker_sp.location.x = walker_params[0]
	walker_sp.location.y = walker_params[1]
	walker_sp.location.z = walker_params[2]
	walker_sp.rotation.pitch = walker_params[3]
	walker_sp.rotation.yaw = walker_params[4]
	walker_sp.rotation.roll = walker_params[5]

	self.walker = world.spawn_actor(walker_bp, walker_sp)
	self.walker_vel.linear.x = walker_params[6] * walker_params[9]
	self.walker_vel.linear.y = walker_params[7] * walker_params[9]

	time.sleep(5)

	#spawn the walker control
	walker_control = carla.WalkerControl(carla.Vector3D(walker_params[6], walker_params[7], walker_params[8]), walker_params[9], False)
	self.walker.apply_control(walker_control)
	world.wait_for_tick()

    def start_scenario(self, scoomatic_goal):
	goal = PoseStamped()
	goal.header.stamp = rospy.Time.now()
	goal.header.frame_id = "map"	
	goal.pose.position.x = scoomatic_goal[0]
	goal.pose.position.y = scoomatic_goal[1]
	goal.pose.position.z = scoomatic_goal[2]
	goal.pose.orientation.x = scoomatic_goal[3]
	goal.pose.orientation.y = scoomatic_goal[4]
	goal.pose.orientation.z = scoomatic_goal[5]
	goal.pose.orientation.w = scoomatic_goal[6]
	self.goal_pub.publish(goal)


    def stop_scenario(self):
	if self.walker is not None:
	    if self.walker.is_alive:
		self.walker.destroy()


    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.scenario_sub.unregister()
	self.stop_scenario()


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("scoomatic_scenario", anonymous=True)
    rate = rospy.Rate(5)

    scoomatic_scenario = None

    try:
        scoomatic_scenario = ScoomaticScenario()
	while not rospy.is_shutdown():
	    if scoomatic_scenario.walker is not None:
		scoomatic_scenario.walker_vel_pub.publish(scoomatic_scenario.walker_vel)
		if scoomatic_scenario.walker.is_alive:
		    scoomatic_pos = scoomatic_scenario.scoomatic_odometry.pose.pose.position
		    # the negation of the y-value is necessary because there is a flipped sign
		    # between the scoomatic and the walker
		    distance = scoomatic_scenario.walker.get_location().distance(carla.Location(scoomatic_pos.x, -scoomatic_pos.y, scoomatic_pos.z))
		    scoomatic_scenario.distance_pub.publish(distance)
	    rate.sleep()
    finally:
        if scoomatic_scenario is not None:
            del scoomatic_scenario
        rospy.loginfo("Done")



if __name__ == '__main__':
    main()
