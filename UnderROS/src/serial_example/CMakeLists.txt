cmake_minimum_required(VERSION 2.8.3)
project(serial_example)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  serial
  std_msgs
)

catkin_package(
  INCLUDE_DIRS  include 
  #LIBRARIES serial_example
  CATKIN_DEPENDS
    serial
    std_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include include/serial_example)
include_directories(
  ${catkin_INCLUDE_DIRS}
)



## Declare a cpp executable
add_executable(serial_example
  src/serialt.cpp
  src/serial_example_node.cpp
)

#add_dependencies(serial_example serial_example_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(serial_example
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS serial_example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

#########




