cmake_minimum_required(VERSION 2.8.3)
project(fast_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(CMAKE_PREFIX_PATH /opt/ros/noetic;/usr/local;$ENV{HOME}/Prometheus/devel/;$ENV{HOME}/planner_ws/devel;$ENV{HOME}/mapping_ws/devel;$ENV{HOME}/mapping_ws/devel_isolated;$ENV{HOME}/tool_ws/devel)
message(STATUS "CMAKE_PREFIX_PATH: ${CMAKE_PREFIX_PATH}")

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  plan_env
  path_searching
  bspline_opt
  traj_utils
  message_generation
        sdf_tools
        sensor_msgs
        rospy
        tf
        visualization_msgs
        nav_msgs
        pcl_ros
        prometheus_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES plan_manage
 CATKIN_DEPENDS plan_env path_searching bspline_opt traj_utils message_runtime pcl_conversions pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  SYSTEM
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        $ENV{HOME}/Prometheus/Modules/common/include
)

add_executable(dyn_planner_node
  src/dyn_planner_node.cpp 
  src/planning_fsm.cpp
  src/dyn_planner_manager.cpp
  src/tools.cpp
  )
target_link_libraries(dyn_planner_node 
  ${catkin_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
  )

add_executable(traj_server 
  src/traj_server.cpp)
target_link_libraries(traj_server
        ${catkin_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
)
#add_dependencies(traj_server ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(test_planning_static
  src/test_planning_static.cpp)
target_link_libraries(test_planning_static 
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
        ${OCTOMAP_LIBRARIES}
  )

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
