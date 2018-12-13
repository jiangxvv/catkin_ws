# toplevel CMakeLists.txt for a catkin workspace
# catkin/cmake/toplevel.cmake

cmake_minimum_required(VERSION 2.8.3)
project("usv_towing")

## Add support for c++11, supported in ROS kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  geometry_msgs)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED)
find_package(cmake_modules REQUIRED)
find_package(Eigen3 REQUIRED)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  actuation.msg
  control.msg 
  course.msg 
  VehicleState.msg 
  
)

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
  std_msgs 
  geometry_msgs
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
  CATKIN_DEPENDS roscpp std_msgs  tf visualization_msgs message_runtime actionlib_msgs
  DEPENDS EIGEN3 Boost
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include/usv_towing
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)


#file(GLOB_RECURSE PATH_PLANNER_FILES src/path_planner/*.cpp)

### Declare a C++ library
#add_library(PathPlanner
#  ${PATH_PLANNER_FILES}
#)

### Add cmake target dependencies of the library
### as an example, code may need to be generated before libraries
### either from message generation or dynamic reconfigure
#add_dependencies(PathPlanner
#  ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}
#)


## Declare a C++ executable
add_executable(MotherShip src/MotherShipDynamicNode.cpp src/MotherShipDynamic.cpp src/MotherShip.cpp)
add_executable(CentralController src/CentralControllerNode.cpp src/CentralController.cpp src/MotherShip.cpp)
add_executable(ThrusterAllocation src/ThrusterAllocation.cpp)
add_executable(Tug1 src/DPtug.cpp src/DPtugDynamic.cpp src/Tug1_Dynamic_Node.cpp)
add_executable(Tug2 src/DPtug.cpp src/DPtugDynamic.cpp src/Tug2_Dynamic_Node.cpp)

#add_executable(viz_ship src/viz_trajectory_node.cpp src/viz_trajectory.cpp)
#add_executable(los_guidance src/los_guidance_node.cpp src/waypoint_tracking_server.cpp)
#add_executable(motion_controller src/motion_controller_node.cpp)
#add_executable(task_manager src/user_interface_node.cpp src/waypoint_tracking_client.cpp src/user_interface.cpp)
#add_executable(thrust_allocation src/thrust_allocation_node.cpp)
#add_executable(path_planner src/path_planner_node.cpp src/path_planner_server.cpp)
#add_executable(sensor_state_publisher src/sensor_state_publisher.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(MotherShip ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(CentralController ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(ThrusterAllocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Tug1 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(Tug2 ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(los_guidance ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(motion_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(task_manager ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(thrust_allocation ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(path_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
#add_dependencies(sensor_state_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Specify libraries to link a library or executable target against
target_link_libraries(MotherShip ${catkin_LIBRARIES})
target_link_libraries(CentralController ${catkin_LIBRARIES})
target_link_libraries(ThrusterAllocation ${catkin_LIBRARIES})
target_link_libraries(Tug1 ${catkin_LIBRARIES})
target_link_libraries(Tug2 ${catkin_LIBRARIES})

#target_link_libraries(los_guidance ${catkin_LIBRARIES} ControlSystem)
#target_link_libraries(motion_controller ${catkin_LIBRARIES} ControlSystem)
#target_link_libraries(task_manager ${catkin_LIBRARIES})
#target_link_libraries(thrust_allocation ${catkin_LIBRARIES} ControlSystem)
#target_link_libraries(path_planner ${catkin_LIBRARIES} PathPlanner)
#target_link_libraries(sensor_state_publisher ${catkin_LIBRARIES})











