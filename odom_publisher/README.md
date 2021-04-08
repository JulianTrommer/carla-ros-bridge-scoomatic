# Odometry Publisher

>This package was created by Niklas Wolf from the University of Augsburg and its functionality is used in this project.

This ROS package creates the tf tree that is needed for the navigation stack.

# Features

- Origin of odometry is set (helper frame)
- Lookup of the origin of odometry to ego vehicle (*odom\_origin -> ego\_vehicle*)
- Transformation set from *odom\_base\_link* to *base\_link*
- Transformation set from *base\_link* to *lidar\_base\_link*
- Transformation set from *map* to *odom\_base\_link*

# Starting the Odometry Publisher

The odometry publisher is started by the packages that need it.

Alternatively you can start the publisher with the following command:
```
roslaunch odom_publisher odom_publisher.launch
```

# ROS Topics

## Subscriber

|Topic                                 | Typ | Beschreibung |
|--------------------------------------|------|-------------|
| `/ego_vehicle/odom_data` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) | Odometrie message of the vehicle |
| `/ego_vehicle/laserscan` | [sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) | LaserScan of the vehicle |
| `/ego_vehicle/lidar` | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | Lidar of the vehicle |

## Publisher

|Topic                                 | Type | Description |
|--------------------------------------|------|-------------|
| `/odom` | [nav_msgs/Odometry](http://docs.ros.org/en/melodic/api/nav_msgs/html/msg/Odometry.html) | Odometrie message of the vehicle |
| `/base_scan_laser` | [sensor_msgs/LaserScan](http://docs.ros.org/en/api/sensor_msgs/html/msg/LaserScan.html) | LaserScan of the vehicle |
| `/base_scan_lidar` | [sensor_msgs/PointCloud2](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/PointCloud2.html) | Lidar of the vehicle |
