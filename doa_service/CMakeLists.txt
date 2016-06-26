cmake_minimum_required(VERSION 2.8.3)
project(doa_service)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy tf std_msgs genmsg message_generation serial)

## Declare ROS messages and services

add_service_files(FILES DoaRssi.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(doa_server src/doa_server.cpp)
target_link_libraries(doa_server ${catkin_LIBRARIES})
add_dependencies(doa_server doa_service_cpp)

add_executable(doa_client src/doa_client.cpp)
target_link_libraries(doa_client ${catkin_LIBRARIES})
add_dependencies(doa_client doa_service_cpp)

#add_executable(add_two_ints_server src/add_two_ints_server.cpp)
#target_link_libraries(add_two_ints_server ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_server beginner_tutorials_gencpp)

#add_executable(add_two_ints_client src/add_two_ints_client.cpp)
#target_link_libraries(add_two_ints_client ${catkin_LIBRARIES})
#add_dependencies(add_two_ints_client beginner_tutorials_gencpp)

