cmake_minimum_required(VERSION 3.0.2)
project(amr_localization)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
    geometry_msgs
    nav_msgs
    roscpp
    tf2
    #amr_custom_interfaces
)
find_package(yaml-cpp REQUIRED)

include_directories(
 ${catkin_INCLUDE_DIRS}
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES amr_localization
    CATKIN_DEPENDS geometry_msgs nav_msgs roscpp
    DEPENDS system_lib
)

add_executable(
  amr_position_helper
  src/amr_position_helper.cpp
)
target_link_libraries(amr_position_helper ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
target_include_directories(amr_position_helper PUBLIC ${catkin_INCLUDE_DIRS} ${YAML_CPP_INCLUDE_DIRS})

#add_executable(amr_wheel_odometry_node src/wheel_odom_node.cpp)
#target_link_libraries(amr_wheel_odometry_node ${catkin_LIBRARIES})

#add_executable(amr_odom_server src/wheel_odom_server.cpp)
#target_link_libraries(amr_odom_server ${catkin_LIBRARIES})
#target_link_directories(amr_odom_server PUBLIC ${catkin_INCLUDE_DIRS})

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch")

install(DIRECTORY params/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/params)


# add_executable(${PROJECT_NAME}_node src/amr_localization_node.cpp)

