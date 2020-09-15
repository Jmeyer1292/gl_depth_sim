cmake_minimum_required(VERSION 3.5.0)
project(roscpp_gl_depth_sim_demos)

if (NOT $ENV{ROS_VERSION} EQUAL "1")
  message(WARNING "ROS Version 1 not found, skipping project ${PROJECT_NAME}")
  return()
endif()

find_package(gl_depth_sim REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  tf2_eigen
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    tf2_ros
    tf2_eigen
    visualization_msgs
  DEPENDS gl_depth_sim
)

add_executable(${PROJECT_NAME}_laser_example src/laser_example.cpp)
set_target_properties(${PROJECT_NAME}_laser_example PROPERTIES OUTPUT_NAME laser_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_laser_example
  ${catkin_LIBRARIES}
  gl_depth_sim::gl_depth_sim_laser_scanner
)

add_executable(${PROJECT_NAME}_camera_example src/camera_example.cpp)
set_target_properties(${PROJECT_NAME}_camera_example PROPERTIES OUTPUT_NAME camera_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_camera_example
  ${catkin_LIBRARIES}
  gl_depth_sim::gl_depth_sim_interfaces
)

install(
  TARGETS
    ${PROJECT_NAME}_camera_example
    ${PROJECT_NAME}_laser_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
