cmake_minimum_required(VERSION 3.0.2)
project(skills_executer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  skills_executer_msgs
  configuration_msgs
  sensor_msgs
  simple_touch_controller_msgs
  actionlib_msgs
  actionlib
  manipulation_msgs
  behaviortree_cpp_v3
  relative_cartesian_controller_msgs
  ur_dashboard_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
                 rospy
                 std_msgs
                 ur_dashboard_msgs
  DEPENDS
)

include_directories(
include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/skills_executer/skills_exec.cpp
  src/skills_executer/bt_skills_classes.cpp
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
)

add_executable(        skills_exec_node src/skills_exec_node.cpp)
add_dependencies(      skills_exec_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries( skills_exec_node
        ${catkin_LIBRARIES}
        ${PROJECT_NAME}
)
