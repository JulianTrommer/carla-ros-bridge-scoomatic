# ROS Scoomatic Launch

This ROS package `carla_scoomatic_launch` can be used to start the scoomatic and its features in a simpler way.

## Starting the node

In order to use this node you need to first start the Carla Simulator in the folder where it is located:
```
./CarlaUE4.sh
```

After that you can use the different launch files provided with this package:

1. Custom launch files for the ros-bridge:
   - carla_ros_bridge_with_custom_rviz.launch:  
     - Starts the ros-bridge and rviz with a custom config file
     - The config contains two cameras of the scoomatic (front and 3rd person) and a 3d-space in which you can see a lidar scan or the result of a mapping package.
   - carla_ros_bridge_with_ego_scoomatic.launch:
     - Starts the ros-bridge with a manually controllable scoomatic equipped with an example array of sensors
2. Mapping-Packages:
   - scoomatic_gmapping.launch:
     - Starts the necessary packages in order to run the gmapping-algorithm.
     - Requires the ros-bridge to be started and an ego-vehicle with a lidar sensor and odometry sensors.