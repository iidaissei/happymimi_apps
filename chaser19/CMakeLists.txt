cmake_minimum_required(VERSION 2.8.3)
project(chaser19)
#Debug option
set(CMAKE_BUILD_TYPE Debug)
#For profilling with gprof
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pg")
SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -pg")
SET(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -pg")
## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs urg_node sensor_msgs joy cv_bridge image_transport) #rospeex_msgs rospeex_if )
find_package(PCL 1.3 REQUIRED COMPONENTS common io )
find_package(OpenCV REQUIRED)

## add demu
include_directories(/usr/local/include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}  )

catkin_package(
  INCLUDE_DIRS
  LIBRARIES chaser19
  CATKIN_DEPENDS roscpp geometry_msgs
  DEPENDS urg_node sensor_msgs roscpp
)

###########
## Build ##
###########

add_executable(chaser19 src/chaser19.cpp)
target_link_libraries(chaser19 ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} )

#############
## Install ##
#############

## Mark executable scripts (Python etc.) for installation
#install(PROGRAMS
#  scripts/move
#  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
#)

## Mark executables and/or libraries for installation
install(TARGETS chaser19
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark all other useful stuff for installation
install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
