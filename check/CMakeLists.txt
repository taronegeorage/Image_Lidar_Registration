cmake_minimum_required(VERSION 2.8.3)
project(check)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)

include_directories(
	include
	${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIR}
  ${Eigen_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
)

add_executable (show_plane check_plane.cpp)
target_link_libraries (show_plane ${PCL_LIBRARIES})

add_executable (check_axis check_axis_pose.cpp)
target_link_libraries (check_axis ${PCL_LIBRARIES})

