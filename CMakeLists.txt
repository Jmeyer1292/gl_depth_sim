cmake_minimum_required(VERSION 2.8.3)
project(gl_depth_sim)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp # Used for ROS examples
  pcl_ros # Used for interfaces extension
  tf2_ros # Used for ROS example
  tf2_eigen # Used for ROS example
  visualization_msgs # Used for ROS example
)

# Required for core functionality
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(Eigen3 REQUIRED)

# Extensions
find_package(assimp REQUIRED) # Just used for loading models in mesh_loader.h
find_package(OpenCV REQUIRED) # Used for interface extension

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_interfaces
  CATKIN_DEPENDS
    roscpp
    pcl_ros
    tf2_ros
    tf2_eigen
    visualization_msgs
  DEPENDS OpenCV
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
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
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}
  ${OPENGL_LIBRARIES}
  ${ASSIMP_LIBRARIES}
  dl
  glfw
)

# Libaries for interfacing with opencv and pcl
add_library(${PROJECT_NAME}_interfaces
  src/interfaces/pcl_interface.cpp
  src/interfaces/opencv_interface.cpp
  src/interfaces/noise_model.cpp
)
add_dependencies(${PROJECT_NAME}_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_interfaces
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Simulated laser scanner library
add_library(${PROJECT_NAME}_sim_laser_scanner
  src/${PROJECT_NAME}/sim_laser_scanner.cpp
)
add_dependencies(${PROJECT_NAME}_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_sim_laser_scanner
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
)

# Example showing basic usage
add_executable(${PROJECT_NAME}_test src/usage_example.cpp)
set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME depth_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${catkin_LIBRARIES}
)

# Example showing an orbiting camera
add_executable(${PROJECT_NAME}_orbit src/camera_orbit_example.cpp)
set_target_properties(${PROJECT_NAME}_orbit PROPERTIES OUTPUT_NAME orbit_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_orbit
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${catkin_LIBRARIES}
)

# Example showing orbiting camera with point cloud publishing in ROS
add_executable(${PROJECT_NAME}_ros_orbit src/camera_ros_example.cpp)
set_target_properties(${PROJECT_NAME}_ros_orbit PROPERTIES OUTPUT_NAME ros_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_ros_orbit
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${catkin_LIBRARIES}
)

# Example showing sweeping laser scan with point cloud publishing in ROS
add_executable(${PROJECT_NAME}_laser_example src/laser_example.cpp)
set_target_properties(${PROJECT_NAME}_laser_example PROPERTIES OUTPUT_NAME laser_example PREFIX "")
target_link_libraries(${PROJECT_NAME}_laser_example
  ${PROJECT_NAME}_sim_laser_scanner
  ${catkin_LIBRARIES}
)

install(
  TARGETS
    ${PROJECT_NAME}
    ${PROJECT_NAME}_interfaces
    ${PROJECT_NAME}_sim_laser_scanner
    ${PROJECT_NAME}_test
    ${PROJECT_NAME}_orbit
    ${PROJECT_NAME}_ros_orbit
    ${PROJECT_NAME}_laser_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

#############
## Testing ##
#############
#if(CATKIN_ENABLE_TESTING)
#  find_package(rostest REQUIRED)
#  catkin_add_gtest(utest_node test/utest.cpp)
#  target_link_libraries(utest_node
#    ${catkin_LIBRARIES}
#    ${PROJECT_NAME}_sim_laser_scanner
#  )
#endif()
