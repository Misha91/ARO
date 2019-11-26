cmake_minimum_required(VERSION 2.8.3)
project(barbie_detection)

find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package()

catkin_install_python(
    PROGRAMS scripts/detector.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
