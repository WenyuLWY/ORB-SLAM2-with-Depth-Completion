cmake_minimum_required(VERSION 3.0.2)
project(slammot)

set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")


find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    rosbag
    message_filters
    
    image_transport
    cv_bridge
    tf

    std_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
)

find_package( OpenCV REQUIRED)
find_package( PCL REQUIRED QUIET)
find_package( Eigen3 REQUIRED)


catkin_package(
 CATKIN_DEPENDS roscpp rospy
 DEPENDS PCL  EIGEN3
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)




add_executable(${PROJECT_NAME}_kittiHelper_node src/kittiHelper.cpp)
add_dependencies(${PROJECT_NAME}_kittiHelper_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_kittiHelper_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES} 
  python2.7
)


