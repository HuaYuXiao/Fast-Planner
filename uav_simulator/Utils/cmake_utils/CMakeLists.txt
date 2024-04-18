cmake_minimum_required(VERSION 2.8.3)
project(cmake_utils)

find_package(catkin REQUIRED COMPONENTS roscpp)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall")

file(GLOB CMAKE_UILTS_FILES cmake/*.cmake)

catkin_package(
  CFG_EXTRAS ${CMAKE_UILTS_FILES}
)

