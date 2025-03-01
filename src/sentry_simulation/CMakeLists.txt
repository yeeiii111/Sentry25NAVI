cmake_minimum_required(VERSION 2.8.3)
project(sentry_simulation)

set(CMAKE_BUILD_TYPE Release)
set(BUILD_STATIC_LIBS ON)
set(BUILD_SHARED_LIBS OFF)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
  livox_ros_driver2
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  CATKIN_DEPENDS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  pcl_ros
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  "${PROJECT_SOURCE_DIR}/include"
  ~/RMUC_simulation/src/sentry_simulation/include
  /usr/local/include # Location when using 'make system_install'
  /usr/include       # More usual location (e.g. when installing using a package)
)

## Specify additional locations for library files
link_directories(
  /usr/local/lib # Location when using 'make system_install'
  /usr/lib       # More usual location (e.g. when installing using a package)
)

## Declare executables
add_executable(vehicle_simulator ${PROJECT_SOURCE_DIR}/src/vehicle_simulator.cpp)
add_executable(key_control ${PROJECT_SOURCE_DIR}/src/key_control.cpp)
add_executable(livox2rosmsg ${PROJECT_SOURCE_DIR}/src/livox2rosmsg.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(vehicle_simulator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(key_control ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(livox2rosmsg ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS vehicle_simulator
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)


install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY log/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/log
)
install(DIRECTORY mesh/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/mesh
)
install(DIRECTORY rviz/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY world/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/world
)

set(PYTHON_EXECUTABLE /usr/bin/python3)
catkin_install_python(PROGRAMS
  src/auto_obstacle.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
