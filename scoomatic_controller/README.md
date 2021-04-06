# ROS Scoomatic Controller

This ROS package `scoomatic_controller` is used by the navigation stack in order to forward the velocity commands to the PID-controller inside the AckermannControl node.

## Starting the node

In order to use this node you need to start CARLA, the ros-bridge with an ego-scoomatic and the navigation base:
```
./CarlaUE4.sh
```
```
roslaunch carla_scoomatic_launch carla_ros_bridge_with_ego_and_rviz.launch
roslaunch scoomatic_navigation move_base.launch
```

>Hint: You can also start the packages (that are used in the carla_scoomatic_launch package) individually.

After that you can simply start the given launch file of this package.

---

Or you can start the full stack inside the scoomatic_scenario package:
```
roslaunch scoomatic_scenario scoomatic_stack_and_scenario.launch
```

