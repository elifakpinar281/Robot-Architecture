cmake_minimum_required(VERSION 3.0.2)
project(turtlebot3_control)

add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  message_generation
  cv_bridge
  std_srvs
  roslib
)

find_package(OpenCV REQUIRED)

add_message_files(
  FILES
  RobotStatus.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs message_runtime cv_bridge std_srvs roslib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(status_publisher src/status_publisher.cpp)
target_link_libraries(status_publisher ${catkin_LIBRARIES})
add_dependencies(status_publisher ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(logger_node src/logger_node.cpp)
target_link_libraries(logger_node ${catkin_LIBRARIES})
add_dependencies(logger_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(camera_node src/camera_node.cpp)
target_link_libraries(camera_node ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})
add_dependencies(camera_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(wall_follower src/wall_follower.cpp)
target_link_libraries(wall_follower ${catkin_LIBRARIES})
add_dependencies(wall_follower ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_between src/move_between.cpp)
target_link_libraries(move_between ${catkin_LIBRARIES})
add_dependencies(move_between ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS
  status_publisher
  logger_node
  camera_node
  wall_follower
  move_between
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch worlds
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)