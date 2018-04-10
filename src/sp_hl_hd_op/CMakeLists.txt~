cmake_minimum_required(VERSION 2.8.3)
project(sp_hl_hd_op)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
	roscpp
	std_msgs
	cv_bridge
	sensor_msgs
	image_transport
	#pub_msgs
	message_generation	
)
# Find OpenNI
find_package(PkgConfig)
pkg_check_modules(OpenNI REQUIRED libopenni)

find_package(OpenCV 2.4.13 REQUIRED)


## Generate messages in the 'msg' folder
 add_message_files(
	FILES
	detection.msg
	humanColor.msg
	pixel_BGR.msg
	detectID.msg
#   Message2.msg
 )


## Generate added messages and services with any dependencies listed here
 generate_messages(
   DEPENDENCIES
   std_msgs  # Or other packages containing msgs
 )


catkin_package(
  INCLUDE_DIRS Include
  LIBRARIES sp_hl_hd_op
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


 include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenNI_INCLUDEDIR}
	${OpenCV_INCLUDE_DIRS}
)


 add_executable(
sp_tracker_local src/sp_tracker_local_roi.cpp

)


 target_link_libraries(sp_tracker_local 

	${catkin_LIBRARIES}
	${OpenNI_LIBRARIES}
	${OpenCV_LIBRARIES}
 )
