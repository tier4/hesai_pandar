cmake_minimum_required(VERSION 3.0.2)
project(pandar_api)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
)

find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
  LIBRARIES
)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

# Target
add_library(pandar_api
  src/tcp_client.cpp
)
target_link_libraries(pandar_api
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)


## add node
add_executable(client
  src/main.cpp
)

target_link_libraries(client
  pandar_api
  ${catkin_LIBRARIES}
)

# Install
## executables and libraries
install(
  TARGETS
    pandar_api
    client
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## project namespaced headers
install(
  DIRECTORY
    include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## directories
install(
  DIRECTORY
    launch
    config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
