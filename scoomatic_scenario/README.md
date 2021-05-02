# ROS Scoomatic Scenario

This ROS package `scoomatic_scenario` can be used to start individual scenarios that test the ability of the scoomatic (in particular the autonomous driving).

## Starting the node

In order to use this node you need to first start the Carla Simulator and the complete navigation stack, including the scoomatic controller:
```
./CarlaUE4.sh
```
```
roslaunch carla_scoomatic_launch carla_scoomatic_with_ego_and_rviz.launch
roslaunch scoomatic_navigation move_base.launch
roslaunch scoomatic_controller scoomatic_controller.launch
```

After that you can use the different the given launch file of this package:
```
roslaunch scoomatic_scenario scoomatic_scenario.launch
```

---

Or you can simply start the full navigation stack with the given launch file:
```
roslaunch scoomatic_scenario scoomatic_stack_and_scenario.launch
```

## Using the node

After you have started the node, you can execute these different scenarios:
  1. The scoomatic and a pedestrian are facing and moving towards each other.
  2. A pedestrian is walking into the driving direction of the scoomatic from the right.
  3. A pedestrian is walking into the driving direction of the scoomatic from the left.

The command for executing a scenario is the following:
<pre>
rostopic pub /scoomatic/scenario std_msgs/UInt8 <i>scenario_id</i>
</pre>
If you want to repeat the previous scenario, simply use 0 as the scenario_id. However, make sure that the scoomatic is standing still, otherwise the end result will be influenced.

>Warning: If you want to use the package "dynamic_obstacle_visualizer" you need to start it before executing a scenario.
