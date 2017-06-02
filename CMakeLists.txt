cmake_minimum_required(VERSION 2.8.3)
project(velo2cam_calibration)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -w ")

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  message_filters
  roscpp
  pcl_ros
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  dynamic_reconfigure
  # geometry_msgs
  # tf2
  # tf2_ros
  # tf2_sensor_msgs
)

find_package(OpenMP)
if(OPENMP_FOUND)
  message(STATUS "OPENMP FOUND")
  set(OpenMP_FLAGS ${OpenMP_CXX_FLAGS})  # or if you use C: ${OpenMP_C_FLAGS}
  set(OpenMP_LIBS gomp)
endif()

find_package(OpenCV 3 REQUIRED)
find_package(PCL 1.7 REQUIRED)


generate_dynamic_reconfigure_options(
  cfg/Laser.cfg
  cfg/Camera.cfg
  cfg/Plane.cfg
)

################################################
## Declare ROS messages, services and actions ##

## Generate messages in the 'msg' folder
add_message_files(
 FILES
 ClusterCentroids.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
 DEPENDENCIES
 std_msgs
 sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_LIBRARIES}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
  #-std=c++11
)

## Declare a cpp executable
add_executable(laser_pattern src/laser_pattern.cpp)
add_executable(stereo_pattern src/stereo_pattern.cpp)
#add_executable(mono_pattern src/mono_pattern.cpp)
add_executable(velo2cam_calibration src/velo2cam_calibration.cpp lib/tinyxml2.cpp lib/tinyxml2.h)
add_executable(pcl_coloring src/pcl_coloring.cpp)
add_executable(pcl_projection src/pcl_projection.cpp)
add_executable(levinson src/levinson.cpp)
add_executable(v2c_plane_segmentation src/plane.cpp)
add_executable(v2c_disp_masker src/disp_masker.cpp)

target_compile_options(pcl_projection PRIVATE ${OpenMP_FLAGS})

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(laser_pattern ${catkin_EXPORTED_TARGETS} velo2cam_calibration_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(stereo_pattern ${catkin_EXPORTED_TARGETS} velo2cam_calibration_generate_messages_cpp ${PROJECT_NAME}_gencfg)
#add_dependencies(mono_pattern ${catkin_EXPORTED_TARGETS})
add_dependencies(velo2cam_calibration ${catkin_EXPORTED_TARGETS} velo2cam_calibration_generate_messages_cpp ${PROJECT_NAME}_gencfg)
add_dependencies(levinson ${catkin_EXPORTED_TARGETS})
add_dependencies(v2c_plane_segmentation ${PROJECT_NAME}_gencfg)

## Specify libraries to link a library or executable target against
target_link_libraries(stereo_pattern
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${PCL_LIBRARIES}
 )
## Specify libraries to link a library or executable target against
target_link_libraries(laser_pattern
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
## Specify libraries to link a library or executable target against
target_link_libraries(velo2cam_calibration
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
## Specify libraries to link a library or executable target against
target_link_libraries(pcl_coloring
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(pcl_projection
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
  ${OpenMP_LIBS}
)
target_link_libraries(levinson
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(v2c_plane_segmentation
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
target_link_libraries(v2c_disp_masker
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)
