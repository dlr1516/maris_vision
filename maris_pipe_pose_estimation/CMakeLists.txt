#
# MARIS Vision - ROS Packages of Vision System used in MARIS project. 
# Copyright (C) 2016 Fabjan Kallasi and Dario Lodi Rizzini.
#
# This file is part of MARIS Vision. 
# Visit http://rimlab.ce.unipr.it/Maris.php for more information about 
# the project. 
#
# MARIS Vision is free software: you can redistribute it and/or modify
# it under the terms of the GNU Lesser General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# MARIS Vision is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.
#
# You should have received a copy of the GNU Lesser General Public License
# along with MARIS Vision.  If not, see <http://www.gnu.org/licenses/>.
#
cmake_minimum_required(VERSION 2.8.3)
project(maris_pipe_pose_estimation)

add_definitions(-std=c++0x)
set(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  visualization_msgs
  sensor_msgs
  std_msgs
  cv_bridge
  message_filters
  image_geometry
  dynamic_reconfigure
  image_proc
maris_ros_msgs
message_generation
  genmsg
)

 add_message_files(
   FILES
   Line3D.msg
 )

 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
   geometry_msgs
 )

generate_dynamic_reconfigure_options(
  cfg/PipePoseEstimationNodeParams.cfg
  cfg/intrinsic_correction.cfg
  cfg/masked_stereo_proc_params.cfg
  cfg/frame_tracker_params.cfg
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs
  sensor_msgs
  visualization_msgs
  std_msgs
  cv_bridge
  message_filters
  image_geometry
  dynamic_reconfigure
  image_proc
  maris_ros_msgs
  message_runtime
  genmsg
#message_runtime
#  DEPENDS system_lib
)


find_package( OpenCV REQUIRED )
#include_directories( ${OpenCV_INCLUDE_DIRS} )
#include_directories(include)

find_package(Boost COMPONENTS lexical_cast)
#include_directories(${Boost_INCLUDE_DIRS})  

link_directories(${PROJECT_SOURCE_DIR}/external/EDLines)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
	${catkin_INCLUDE_DIRS}
	include
	external/EDLines
	${OpenCV_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(PipePoseEstimationNode src/PipePoseEstimationNode.cpp src/CylinderLSEstimator.cpp src/Profiler.cpp)
add_dependencies(PipePoseEstimationNode ${PROJECT_NAME}_gencfg ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(PipePoseEstimationNode ${catkin_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} EDLines)

#add_executable(stereo_reader_node src/stereo_reader_node.cpp)
#add_dependencies(stereo_reader_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(stereo_reader_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(ArmReprojectorNode src/ArmReprojectorNode.cpp)
target_link_libraries(ArmReprojectorNode ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(fake_wTv_node src/fake_wTv_node.cpp)
target_link_libraries(fake_wTv_node ${catkin_LIBRARIES})

#add_executable(stereo_proc_node src/stereo_proc_node.cpp src/stereo_proc_core.cpp src/stereo_bm.cpp)
#add_dependencies(stereo_proc_node ${PROJECT_NAME}_gencfg)
#target_link_libraries(stereo_proc_node ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(pipe_tracker_node src/pipe_tracker_node.cpp src/PipePoseTracker.cpp src/FrameTracker.cpp)
target_link_libraries(pipe_tracker_node ${catkin_LIBRARIES})

add_executable(frame_tracker_node src/frame_tracker.cpp src/FrameTracker.cpp)
target_link_libraries(frame_tracker_node ${catkin_LIBRARIES})

