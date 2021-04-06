# ROS Dynamic Obstacle Visualizer

This ROS package `dynamic_obstacle_visualizer` can be used to
  - convert the obstacles of the costmap into dynamic obstacles for pose prediction.
  - visualize the estimated obstacle velocity and estimated pose.

## Starting the node

In order to use this node you need to start the full navigation stack (start CARLA and the ros-bridge with an ego-scoomatic), including the custom scenario runner:
```
./CarlaUE4.sh
```
```
roslaunch scoomatic_scenario scoomatic_stack_and_scenario.launch
```

>Hint: You can also start the packages (that are used in the scoomatic_scenario package) individually.

After that you can simply start the given launch file of this package.