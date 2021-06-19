cmake_minimum_required(VERSION 2.8.3)
project(rs_command)

find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
std_msgs
geometry_msgs
message_generation)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})

# catkin_install_python(
# 	PROGRAMS
# 	  scripts/straight_line_2_the_goal.py
#     DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

add_service_files(
  FILES
  ReqWayPoints.srv
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package()

# add_executable(path_publisher src/path_publisher.cc)
# target_link_libraries(path_publisher ${GAZEBO_LIBRARIES})

# add_library(model_move SHARED src/model_move.cc)
# target_link_libraries(model_move ${GAZEBO_LIBRARIES})

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})