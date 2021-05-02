#!/usr/bin/env python
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

import os
import sys
import time
import operator

import rospy
import carla

from std_msgs.msg import Float32
from custom_msgs.msg import ScenarioParams
from actionlib_msgs.msg import GoalStatusArray, GoalID
from carla_msgs.msg import CarlaCollisionEvent
from std_srvs.srv import Empty

sys.path.append(os.path.abspath(os.path.join(os.path.dirname( __file__ ), "..", "..", "..", "..", "..", "..", "carla-ros-bridge-scoomatic", "scenario_optimizer", "BayesianOptimization", "bayes_opt")))
from bayesian_optimization import BayesianOptimization
from domain_reduction import SequentialDomainReductionTransformer

from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt

# ==============================================================================
# ---ScenarioOptimizer----------------------------------------------------------
# ==============================================================================


class ScenarioOptimizer(object):

    def __init__(self):
        self.role_name = rospy.get_param("~role_name", 'ego_vehicle')

	self.client = carla.Client(
	    rospy.get_param("~host", "localhost"), rospy.get_param("~port", "2000"))
	self.client.set_timeout(10.0)

	# the time at which the current scenario started in nanoseconds
	self.scenario_start_time = rospy.Time.now()
	self.current_scenario = ""
	self.current_goal = None
	self.scenario_running = False
	self.xyd = []
	self.last_minimum_distance = 0.0
	self.collision_occured = False

        # ==========================================
        # -- Subscriber ----------------------------
        # ==========================================

	# scenario message for selecting the executed scenario
	self.distance = Float32()
	self.distance_sub = rospy.Subscriber("distance_to_walker", Float32, self.distance_received)

	self.nav_status_sub = rospy.Subscriber("move_base/status", GoalStatusArray, self.nav_status_updated)

	self.collision_sub = rospy.Subscriber("carla/ego_vehicle/collision", CarlaCollisionEvent, self.collision_detected)

        # ==========================================
        # -- Publisher ----------------------------
        # ==========================================

	# a list containing the executed scenarios and their status
	self.nav_dict = {}
	self.scenario_pub = rospy.Publisher("/scoomatic/scenario", ScenarioParams, queue_size=50, latch=True)

	self.cancel_pub = rospy.Publisher("move_base/cancel", GoalID, queue_size=50)


    class NavStatus:

	def __init__(self):
	    self.status = 0
	    self.distance_values = {}

	def set_status(self, status):
	    self.status = status

	def get_status(self):
	    return self.status

	def add_value(self, timestamp, distance):
	    self.distance_values[timestamp] = distance

	def get_values(self):
	    return self.distance_values


    def distance_received(self, distance):
	if self.current_scenario != "" and self.current_scenario in self.nav_dict:
	    timestamp = rospy.Time.now() - self.scenario_start_time
	    self.nav_dict[self.current_scenario].add_value(timestamp, distance.data)


    def nav_status_updated(self, update):
	if len(update.status_list) > 0:
	    self.current_goal = update.status_list[-1]
	    if self.current_goal.status < 2 and not self.collision_occured:
		self.current_scenario = self.current_goal.goal_id.id
		self.scenario_start_time = self.current_goal.goal_id.stamp
		if self.current_scenario not in self.nav_dict:
		    self.nav_dict[self.current_scenario] = self.NavStatus()
		    self.nav_dict[self.current_scenario].set_status(self.current_goal.status)
	    elif (self.current_goal.status > 1 or self.collision_occured) and self.current_scenario != "":
		if not self.collision_occured:
		    min_dist = min(self.nav_dict[self.current_goal.goal_id.id].get_values().iteritems(), key=operator.itemgetter(1))[1]
		    self.last_minimum_distance = min_dist
		else:
		    self.last_minimum_distance = 0.0
		# The current goal is neither pending (=0) nor active (=1)
		time.sleep(15)
		# reset the scenario
		clear_cm = rospy.ServiceProxy("/move_base/clear_costmaps", Empty)
		clear_cm()
		self.current_scenario = ""
		self.scenario_running = False
		self.collision_occured = False


    def generate_scenario_msg(self, scoomatic_pose, walker_params, scoomatic_goal):
	params = ScenarioParams()
	params.scoomatic_pose = scoomatic_pose
	params.walker_params = walker_params
	params.scoomatic_goal = scoomatic_goal
	return params

    def __del__(self):
        """
        Unregister all publisher and subscriber
        """
        rospy.loginfo("Unregister topics...")
        self.distance_sub.unregister()
	self.nav_status_sub.unregister()
	self.collision_sub.unregister()

    def collision_detected(self, collision):
	if not self.collision_occured:
	    rospy.loginfo("Collision detected! Aborting current scenario...")
	    self.collision_occured = True
	    if self.current_scenario == "":
		self.current_scenario = self.current_goal.goal_id
	    self.cancel_pub.publish(self.current_goal.goal_id)


    def run_scenario(self, walk_x, walk_y):
	scoomatic_pose = [-18.85, 49.13, -1.4, 0.00, 0.00, 0.71, 0.70]
    	walker_params = [-19.25, -79.60, 1.3, 0.0, 90.0, 0.0, walk_x, walk_y, 0.0, 0.5]
    	scoomatic_goal = [-19.25, 79.60, 1.3, 0.00, 0.00, 0.71, 0.70]
    	params = self.generate_scenario_msg(scoomatic_pose, walker_params, scoomatic_goal)
    	self.scenario_pub.publish(params)
	self.scenario_running = True
	while self.scenario_running:
	    pass
	self.xyd.append([walk_x, walk_y, self.last_minimum_distance])
	return self.last_minimum_distance


    def plot_graph(self):
	fig = plt.figure()
	ax = plt.axes(projection="3d")
	

	x_points = [row[0] for row in self.xyd]
	y_points = [row[1] for row in self.xyd]
	z_points = [row[2] for row in self.xyd]

	ax.scatter3D(x_points, y_points, z_points)

	ax.set_xlabel("walk_x")
	ax.set_ylabel("walk_y")
	ax.set_zlabel("distance")
	plt.show()



    def print_result(self):
	print("Scenarios:")
	for key in self.nav_dict:
	    value = self.nav_dict[key]
	    print("  ID: {id}".format(id=key))
	    print("  Status: {status}".format(status=value.get_status()))
	    print("  Distances:")
	    points = value.get_values()
	    for k in sorted(points):
		print(type(k))
		print("    Timestamp: " + str(k))
		print("    Distance: {distance}m".format(distance=points[k]))
	    print("------------------")


# ==============================================================================
# -- main() --------------------------------------------------------------------
# ==============================================================================


def main():
    """
    main function
    """
    rospy.init_node("scenario_optimizer", anonymous=True)

    scenario_optimizer = ScenarioOptimizer()

    pbounds = {"walk_x": (-0.1, 0.1), "walk_y": (0.75, 1.25)}
    bounds_transformer = SequentialDomainReductionTransformer()

    optimizer = BayesianOptimization(f=scenario_optimizer.run_scenario, pbounds=pbounds, verbose=2, random_state=1, bounds_transformer=bounds_transformer)
    
    optimizer.probe(params={"walk_x": 0.0, "walk_y": 1.0}, lazy=True)

    optimizer.maximize(init_points=10, n_iter=20)

    scenario_optimizer.plot_graph()
    
    if scenario_optimizer is not None:
	#scenario_optimizer.print_result()
	del scenario_optimizer
    rospy.loginfo("Done")


if __name__ == '__main__':
    main()
