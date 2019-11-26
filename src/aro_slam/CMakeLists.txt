cmake_minimum_required(VERSION 2.8.3)
project(aro_slam)

find_package(catkin REQUIRED)

# For aro_slam package
catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
        geometry_msgs
        laser_geometry
        message_runtime
        nav_msgs
        point_cloud_color
        rospy
        sensor_msgs
        tf2_ros
        tf2_sensor_msgs
)

catkin_install_python(
    PROGRAMS scripts/icp_slam_2d
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
