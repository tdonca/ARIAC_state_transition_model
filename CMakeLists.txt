cmake_minimum_required(VERSION 2.8.3)
project(state_transitions)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(Eigen3 REQUIRED)
if(NOT EIGEN3_INCLUDE_DIRS)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  osrf_gear
  roscpp
  roslaunch
  sensor_msgs
  std_srvs
  trajectory_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
  geometry_msgs 
  pluginlib
  geometric_shapes
)


## Check .launch files for proper format in the launch/ folder
roslaunch_add_file_check(launch)


###################################
## catkin specific configuration ##
###################################
catkin_package(
   CATKIN_DEPENDS moveit_core moveit_ros_planning_interface
   DEPENDS EIGEN3
)





###########
## Build ##
########### 
include_directories(
 include
 ${catkin_INCLUDE_DIRS} 
 ${Eigen_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})


add_executable(first_step src/first_step.cpp)
add_executable(second_step src/second_step.cpp)

add_dependencies(first_step ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(second_step ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(first_step ${catkin_LIBRARIES})
target_link_libraries(second_step ${catkin_LIBRARIES})






