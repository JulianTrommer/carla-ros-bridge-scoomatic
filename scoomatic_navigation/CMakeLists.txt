cmake_minimum_required(VERSION 2.8.3)
project(scoomatic_navigation)

find_package(catkin REQUIRED COMPONENTS rospy roslaunch)

roslaunch_add_file_check(launch IGNORE_UNSET_ARGS)

catkin_package(CATKIN_DEPENDS rospy)

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)

install(DIRECTORY config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config)
