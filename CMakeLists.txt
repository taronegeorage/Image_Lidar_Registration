cmake_minimum_required(VERSION 2.8.3)
project(image_lidar_registration)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
#find_package(Eigen REQUIRED)

include_directories(
	include
	${PCL_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
	#${GTSAM_INCLUDE_DIR}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
	${PCL_LIBRARY_DIRS}
	#${GTSAM_LIBRARY_DIRS}
)


add_executable(projection src/projection_node.cpp src/fisheyeimg.cpp)
target_link_libraries(projection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(extract_indices src/extract_indices.cpp)
target_link_libraries(extract_indices ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (planar_segmentation src/planar_segmentation.cpp)
target_link_libraries (planar_segmentation ${PCL_LIBRARIES})

add_executable (plane_tracking1 src/plane_tracking1.cpp src/fisheyeimg.cpp src/utils/tracking.cpp)
target_link_libraries (plane_tracking1 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (plane_tracking2 src/plane_tracking2.cpp src/fisheyeimg.cpp src/utils/tracking.cpp)
target_link_libraries (plane_tracking2 ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable (reconstruct src/reconstruction.cpp src/fisheyeimg.cpp)
target_link_libraries (reconstruct ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})