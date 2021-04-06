# ROS Scoomatic Navigation

This ROS package `scoomatic_navigation` can be used to start the navigation stack of the scoomatic, including costmaps and the planners.

## Starting the node

In order to use this node you need to start CARLA and the ros-bridge with an ego-scoomatic:
```
./CarlaUE4.sh
```
```
roslaunch carla_scoomatic_launch carla_ros_bridge_with_ego_and_rviz.launch
```

>Hint: You can also start the packages (that are used in the carla_scoomatic_launch package) individually.

After that you can use the given launch file of this package.

---

Or you can start the full stack inside the scoomatic_scenario package:
```
roslaunch scoomatic_scenario scoomatic_stack_and_scenario.launch
```
