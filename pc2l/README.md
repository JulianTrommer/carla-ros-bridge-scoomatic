# PointCloud to Laserscan

This package converts the incoming PointCloud data into LaserScan messages. It simply starts the ros package (*pointcloud\_to\_laserscan*)[http://wiki.ros.org/pointcloud_to_laserscan] and changes the parameters of it.

# Setup

In order to use this package you need to install the package *pointcloud\_to\_laserscan*:

```
sudo apt install ros-melodic-pointcloud-to-laserscan
```

# Starting the node

The odometry publisher is started by the packages that need it.

Alternatively you can start the publisher with the following command:

```
roslaunch pc2l pointcloud_to_laserscan.launch
```