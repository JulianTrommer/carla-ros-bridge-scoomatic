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

After that you can use the given launch file of this package:
```
roslaunch scoomatic_scenario scoomatic_scenario.launch
```

---

Or you can simply start the full navigation stack with the given launch file:
```
roslaunch scoomatic_scenario scoomatic_stack_and_scenario.launch
```

## Using the node

After you have started the node, you can publish a rostopic with the parameters defined by the [ScenarioParams](../custom_msgs/msg/ScenarioParams.msg) message:
<pre>
rostopic pub /scoomatic/scenario custom_msgs/ScenarioParams <i>scenario_params</i>
</pre>

or you can use the [scenario_optimizer](../scenario_optimizer/) package that automatically starts the scenarios.

>Warning: If you want to use the package "dynamic_obstacle_visualizer" you need to start it before executing a scenario.
