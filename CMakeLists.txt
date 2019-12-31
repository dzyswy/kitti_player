cmake_minimum_required(VERSION 2.8.7)
project(kitti CXX)

# ---[ Using cmake scripts and modules
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/Modules)

#SET (CMAKE_BUILD_TYPE Release)
SET (CMAKE_CXX_FLAGS "-O2 -std=c++11 -Wall")
if (UNIX)
	add_definitions(-fPIC)
	add_definitions(-fpermissive)
endif()	
 
 
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/libs)

find_package(catkin REQUIRED COMPONENTS
                    roscpp
					pcl_conversions 
					pcl_ros 
                    tf
                    tf2
                    std_msgs                    
                    geometry_msgs
                    cv_bridge
                    image_transport
                    dynamic_reconfigure
)

#SET(PCL_DIR /opt/src/pcl/libpcl-1.8.1-x86/share/pcl-1.8/)
#find_package(PCL 1.8 REQUIRED)
find_package(PCL REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system program_options)
find_package(OpenCV REQUIRED)
find_package(Glog REQUIRED)
if (NOT GLOG_FOUND)
	message(STATUS "    Failed to find glog")
else()
	message(STATUS "    include: ${GLOG_LIBRARIES}")
endif()
find_package(Protobuf REQUIRED)
if (NOT PROTOBUF_FOUND)
	message(STATUS "    Failed to find protobuf")
else()
	message(STATUS "    include: ${PROTOBUF_LIBRARIES}")
endif()

include_directories(
		${}
  		${catkin_INCLUDE_DIRS} 
  		${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
		${OpenCV_INCLUDE_DIRS}
		${GLOG_INCLUDE_DIRS}
		${PROTOBUF_INCLUDE_DIRS}
)


include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${PROJECT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR}/proto)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(LIBS ${catkin_LIBRARIES}) 
set(LIBS ${LIBS} ${PCL_LIBRARIES}) 
set(LIBS ${LIBS} ${Boost_LIBRARIES}) 
set(LIBS ${LIBS} ${OpenCV_LIBS}) 
set(LIBS ${LIBS} ${GLOG_LIBRARIES})
set(LIBS ${LIBS} ${PROTOBUF_LIBRARIES})


message(STATUS "LIBS:    ${LIBS}")

#add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/include/kitti)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/proto)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/src/kitti)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/example)



message(STATUS "Using install prefix: ${CMAKE_INSTALL_PREFIX}")











