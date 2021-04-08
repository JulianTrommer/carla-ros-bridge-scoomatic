# ROS Scoomatic

>This README and parts of the package were derived from the carla_ego_vehicle package of the carla-ros-bridge repository (Github page: https://github.com/carla-simulator/ros-bridge/tree/0.9.8/carla_ego_vehicle)

The reference CARLA client `carla_scoomatic` can be used to spawn a scoomatic with attached sensors.

If no specific position is set, the scoomatic is spawned at a random position.

## Spawning at specific position

It is possible to (re)spawn the scoomatic at the specific location by publishing to `/carla/<ROLE NAME>/initialpose`.

The preferred way of doing that is using RVIZ.

Selecting a Pose with '2D Pose Estimate' will delete the current scoomatic and respawn it at the specified position.

## Re-use existing scoomatic

It is possible to re-use an existing scoomatic, instead of spawning a new scoomatic. In this case, the role_name is used to identify the scoomatic
among all CARLA actors through the rolename attribute. Upon success, the requested sensors are attached to this actor, and the actor becomes the new scoomatic.

To make use of this option, set the ROS parameter spawn_scoomatic to false.

## Create your own sensor setup

Sensors, attached to the scoomatic can be defined via a json file. `carla_scoomatic` reads it from the file location defined via the private ros parameter `sensor_definition_file`.

The format is defined like that:

    { "sensors" = [
        {
          "type": "<SENSOR-TYPE>",
          "id": "<NAME>",
          "x": 0.0, "y": 0.0, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, # pose of the sensor, relative to the vehicle
          <ADDITIONAL-SENSOR-ATTRIBUTES>
        },
        ...
      ]
    }

Define sensors with their attributes as described in the Carla Documentation about [Sensors reference](https://github.com/carla-simulator/carla/blob/dev/Docs/ref_sensors.md).

An example is provided by [carla_example_scoomatic.launch](launch/carla_example_scoomatic.launch). It uses the sensors from [sensors.json](config/sensors.json)

## Starting the node

In order to start this node you need to first start the Carla Simulator in the folder where it is located:
```
./CarlaUE4.sh
```

After that you have two different approaches to use the package:

### Use individual launch files:

1. First you can either start the standard ros-bridge:
   ```
   roslaunch carla_ros_bridge carla_ros_bridge.launch
   ```
   or use the custom launch-file with a preconfigured config file for rviz:
   ```
   roslaunch carla_scoomatic carla_ros_bridge_with_custom_rviz.launch
   ```
2. Then you can create a scoomatic with your own custom sensor file:
    ```
    roslaunch carla_scoomatic carla_scoomatic.launch
    ```
    or use the example config given in the node:
    ```
    roslaunch carla_scoomatic carla_example_scoomatic.launch
    ```

Please note that in order to see the output of your sensors in rviz if you use the individual launch files you have to change the topics of the displays. The reason is that the vehicle is not treated as the user's vehicle and it is therefore given a incremental id which is used in the name of the topic.

### Use the provided ego-vehicle:

If you want to use the predefined ego-vehicle you can simply use the given launch-file:
```
roslaunch carla_scoomatic carla_ros_bridge_with_ego_scoomatic.launch
```

In order to use the custom rviz-config you can start rviz with the file as a parameter:
```
rosrun rviz rviz -d (path_to_your_ros_bridge_folder)/carla-ros-bridge-scoomatic/carla_scoomatic/config/rviz_scoomatic_config.rviz
```