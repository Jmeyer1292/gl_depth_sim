cmake_minimum_required(VERSION 2.8.3)
project(gl_depth_sim)

add_compile_options(-std=c++11 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  pcl_ros
)

# Required for core functionality
find_package(glfw3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(assimp REQUIRED) # Just used for loading models in mesh_loader.h
find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} ${PROJECT_NAME}_interfaces
  CATKIN_DEPENDS roscpp
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
)

add_dependencies(${PROJECT_NAME}_interfaces ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_interfaces
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

# Test node for devel purposes
add_executable(${PROJECT_NAME}_test src/usage_example.cpp)

set_target_properties(${PROJECT_NAME}_test PROPERTIES OUTPUT_NAME depth_example PREFIX "")

target_link_libraries(${PROJECT_NAME}_test
  ${PROJECT_NAME}
  ${PROJECT_NAME}_interfaces
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
