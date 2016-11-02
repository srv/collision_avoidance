cmake_minimum_required(VERSION 2.8.3)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules/")

project(collision_avoidance)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  sensor_msgs
  image_transport
  message_filters
  cv_bridge
  pcl_ros
  message_generation
  cmake_modules)

add_message_files(
  DIRECTORY msg
  FILES ObstacleInfo.msg
)
generate_messages(
  DEPENDENCIES std_msgs
)

####################################################
# Dependencies:
find_package(PkgConfig)

# Dependencies - Eigen:
find_package(Eigen3 REQUIRED)

# Dependencies - PCL:
find_package(PCL REQUIRED)

# Dependencies - OpenCV:
find_package(OpenCV REQUIRED)
link_directories(${OpenCV_LIBRARY_DIRS})

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS})

# Stereo obstacle detector node
add_executable(stereo_obstacle_detector
  src/stereo_obstacle_detector.cpp)
target_link_libraries(stereo_obstacle_detector
  ${EIGEN3_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES})
add_dependencies(stereo_obstacle_detector ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})