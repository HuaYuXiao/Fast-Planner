cmake_minimum_required(VERSION 2.8.3)
project(plan_env)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -O3 -Wall")

add_subdirectory(${PROJECT_SOURCE_DIR}/ThirdParty/arc_utilities)
add_subdirectory(${PROJECT_SOURCE_DIR}/ThirdParty/sdf_tools)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  tf
  octomap_msgs
  visualization_msgs
        sdf_tools
        arc_utilities
        sensor_msgs
        geometry_msgs
        nav_msgs
        pcl_ros
        prometheus_msgs
)
find_package(Boost REQUIRED COMPONENTS system)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(octomap REQUIRED)
find_package(sdf_tools REQUIRED)
find_package(arc_utilities REQUIRED)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIRS} 
    ${PCL_INCLUDE_DIRS}
        ${OCTOMAP_INCLUDE_DIRS}
        $ENV{HOME}/Prometheus/Modules/common/include
        ${PROJECT_SOURCE_DIR}/ThirdParty/sdf_tools/include
        ${PROJECT_SOURCE_DIR}/ThirdParty/arc_utilities/include
)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES plan_env
        CATKIN_DEPENDS roscpp std_msgs tf arc_utilities sdf_tools
        #  DEPENDS system_lib
)

link_directories(${PCL_LIBRARY_DIRS})

add_library(plan_env
    src/sdf_map.cpp 
    src/edt_environment.cpp
    src/global_point_sdf.cpp
    )
target_link_libraries(plan_env
    ${catkin_LIBRARIES} 
    ${PCL_LIBRARIES}
    ${OCTOMAP_LIBRARIES}
    )  

add_executable(read_pub_ot
        src/read_pub_ot.cpp)
target_link_libraries(read_pub_ot
        ${OCTOMAP_LIBRARIES}
        ${catkin_LIBRARIES}
        )
