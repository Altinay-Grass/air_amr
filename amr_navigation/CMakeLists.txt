cmake_minimum_required(VERSION 3.0.2)
project(amr_navigation)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    actionlib
    move_base_msgs
)
find_package(yaml-cpp REQUIRED) 

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES amr_navigation
 CATKIN_DEPENDS roscpp actionlib move_base_msgs
 DEPENDS system_lib
)

include_directories(
    include
)

add_executable(
    amr_nav_client
    src/amr_nav_client.cpp
)
target_link_libraries(
    amr_nav_client
    ${catkin_LIBRARIES}
    ${YAML_CPP_LIBRARIES}
)
target_include_directories(
    amr_nav_client
    PUBLIC
    ${catkin_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")
