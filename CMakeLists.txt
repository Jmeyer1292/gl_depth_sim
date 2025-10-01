cmake_minimum_required(VERSION 3.8)
project(gl_depth_sim)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra)

# Used for ROS examples
find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED QUIET)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

# Required for core functionality
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

# Extensions
find_package(assimp REQUIRED) # Just used for loading models in mesh_loader.h
find_package(OpenCV REQUIRED) # Used for interface extension

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# Primary rendering library
# Independent of ROS, but does need glfw3 and assimp for model loading
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/sim_depth_camera.cpp
  src/${PROJECT_NAME}/mesh_loader.cpp
  src/${PROJECT_NAME}/mesh.cpp
  src/${PROJECT_NAME}/renderable_mesh.cpp
  src/${PROJECT_NAME}/shader_program.cpp
  src/${PROJECT_NAME}/glad.c
  src/${PROJECT_NAME}/glfw_guard.cpp
)
target_link_libraries(${PROJECT_NAME}
  ${OPENGL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  dl
  glfw
)
# ament_target_dependencies(${PROJECT_NAME}
#   rclcpp
#   pcl_conversions
#   pcl_ros
#   tf2_ros
#   tf2_eigen
#   visualization_msgs
#   OpenCV
# )

# Libraries for interfacing with opencv and pcl
add_library(${PROJECT_NAME}_interfaces
  src/interfaces/pcl_interface.cpp
  src/interfaces/opencv_interface.cpp
)
target_link_libraries(${PROJECT_NAME}_interfaces
  ${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
)
ament_target_dependencies(${PROJECT_NAME}_interfaces
  rclcpp
  pcl_conversions
  pcl_ros
  tf2_ros
  tf2_eigen
  visualization_msgs
  OpenCV
)

# Simulated laser scanner library
add_library(${PROJECT_NAME}_sim_laser_scanner
  src/${PROJECT_NAME}/sim_laser_scanner.cpp
)
target_link_libraries(${PROJECT_NAME}_sim_laser_scanner
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
)

# Example showing basic usage
add_executable(${PROJECT_NAME}_test src/usage_example.cpp)
set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME depth_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
)

# Example showing an orbiting camera
add_executable(${PROJECT_NAME}_orbit src/camera_orbit_example.cpp)
set_target_properties(${PROJECT_NAME}_orbit PROPERTIES OUTPUT_NAME orbit_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_orbit
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
)

# Example showing orbiting camera with point cloud publishing in ROS
add_executable(${PROJECT_NAME}_ros_orbit src/camera_ros_example.cpp)
set_target_properties(${PROJECT_NAME}_ros_orbit PROPERTIES OUTPUT_NAME ros_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_ros_orbit
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
)

# Example showing sweeping laser scan with point cloud publishing in ROS
add_executable(${PROJECT_NAME}_laser_example src/laser_example.cpp)
set_target_properties(${PROJECT_NAME}_laser_example PROPERTIES OUTPUT_NAME laser_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_laser_example
  ${PROJECT_NAME}_sim_laser_scanner
)

# Install targets
install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_interfaces
    ${PROJECT_NAME}_sim_laser_scanner
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}_orbit
    ${PROJECT_NAME}_ros_orbit
    ${PROJECT_NAME}_laser_example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME})

install(DIRECTORY launch test
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
