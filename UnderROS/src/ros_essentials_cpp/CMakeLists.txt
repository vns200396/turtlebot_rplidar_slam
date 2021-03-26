cmake_minimum_required(VERSION 3.0.2)
project(ros_essentials_cpp)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  dynamic_reconfigure
)


## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
#)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )



## Generate dynamic reconfigure parameters in the 'cfg' folder
generate_dynamic_reconfigure_options(
cfg/chapter2.cfg
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_essentials_cpp
  CATKIN_DEPENDS roscpp rospy std_msgs  
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#talker
add_executable(talker_node src/talker_listener/talker.cpp)
target_link_libraries (talker_node ${catkin_LIBRARIES})
add_dependencies(talker_node  ros_essentials_cpp_gencfg)

#listener
add_executable(listener_node src/talker_listener/listener.cpp)
target_link_libraries (listener_node ${catkin_LIBRARIES})




