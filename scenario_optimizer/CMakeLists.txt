cmake_minimum_required(VERSION 2.8.3)
project(scenario_optimizer)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch custom_msgs actionlib_msgs carla_msgs std_srvs)

roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)

catkin_package(CATKIN_DEPENDS rospy)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
