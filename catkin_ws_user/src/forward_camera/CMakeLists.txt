cmake_minimum_required( VERSION 2.8.3 )
project( forward_camera )


find_package(catkin REQUIRED COMPONENTS
sensor_msgs
cv_bridge
roscpp
std_msgs
image_transport
)

add_compile_options(-std=c++11)

find_package( OpenCV REQUIRED )


catkin_package()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable( live_pattern_calibration src/live_pattern_calibration.cpp )
target_link_libraries( live_pattern_calibration ${catkin_LIBRARIES} ${OpenCV_LIBS} )

add_executable( undistort_camera src/undistort_camera.cpp )
target_link_libraries( undistort_camera ${catkin_LIBRARIES} ${OpenCV_LIBS} )
