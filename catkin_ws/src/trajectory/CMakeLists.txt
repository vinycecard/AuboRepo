cmake_minimum_required(VERSION 2.8.3)
project(trajectory)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  sensor_msgs
  genmsg
  actionlib_msgs
  actionlib
  #aubo_msgs
  aubo_driver
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need

catkin_package(
INCLUDE_DIRS include
LIBRARIES trajectory
CATKIN_DEPENDS roscpp rospy std_msgs trajectory_msgs sensor_msgs actionlib_msgs actionlib aubo_driver
)



###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(include
#  ../aubo_robot/aubo_driver/include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/trajectory.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/trajectory_node.cpp)

add_executable(trajectory_publisher
src/trajectory_publisher.cpp
src/tinystr.cpp
src/tinyxmlerror.cpp
src/tinyxmlparser.cpp
src/tinyxml.cpp)
target_link_libraries(trajectory_publisher ${catkin_LIBRARIES})

add_executable(grid_trajectory
src/grid_trajectory.cpp
src/tinystr.cpp
src/tinyxmlerror.cpp
src/tinyxmlparser.cpp
src/tinyxml.cpp)
target_link_libraries(grid_trajectory ${catkin_LIBRARIES})

add_executable(initial_conditions
src/initial_conditions.cpp)
target_link_libraries(initial_conditions ${catkin_LIBRARIES})

add_executable(xml_on_the_fly
src/xml_on_the_fly.cpp
src/tinystr.cpp
src/tinyxmlerror.cpp
src/tinyxmlparser.cpp
src/tinyxml.cpp)
target_link_libraries(xml_on_the_fly ${catkin_LIBRARIES})

add_executable(filter
src/filter.cpp)
target_link_libraries(filter ${catkin_LIBRARIES})

add_executable(joint_control
src/joint_control.cpp)
target_link_libraries(joint_control ${catkin_LIBRARIES})

add_executable(openloop_control
src/openloop_control.cpp)
target_link_libraries(openloop_control ${catkin_LIBRARIES})# libauborobotcontroller.so libotgLib.a) #error: cant find

add_executable(velocity_estimator
src/velocity_estimator.cpp)
target_link_libraries(velocity_estimator ${catkin_LIBRARIES})

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
install(DIRECTORY include/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_trajectory.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
