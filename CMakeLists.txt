cmake_minimum_required(VERSION 2.8.3)
project(Tracking)


## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg)

find_package(OpenCV REQUIRED)
if(NOT OpenCV_FOUND)
    message(FATAL_ERROR "\n\n OpenCV2 is missing")
endif()


find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing")
endif()

add_compile_options(-std=c++11)
## SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGE} -std=c++11 -pthread")

SET(SRC_LIST src/main.cpp src/tracking.cpp src/fhog.cpp src/kcftracker.cpp)

## Declare ROS messages and services
add_message_files(FILES tracking.msg setpoint.msg)
#add_service_files(FILES AddTwoInts.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package()


## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories( ${realsense_INCLUDE_DIR})

#add_executable(auxiliary src/main.cpp src/datapackage.cpp src/arm.cpp)
add_executable(kcf ${SRC_LIST})
target_link_libraries(kcf ${catkin_LIBRARIES} ${DEPENDENCIES} ${realsense2_LIBRARY} ${OpenCV_LIBS})
#add_dependencies(auxiliary beginner_tutorials_generate_messages_cpp)

#add_executable(listener src/listener.cpp)
#target_link_libraries(listener ${catkin_LIBRARIES})
#add_dependencies(listener beginner_tutorials_generate_messages_cpp)
