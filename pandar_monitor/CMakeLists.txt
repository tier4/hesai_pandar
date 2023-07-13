cmake_minimum_required(VERSION 3.5)
project(pandar_monitor)

### Compile options
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-unused-parameter -Wall -Wextra -Wpedantic -Werror)
endif()

### Dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

include_directories(
  include
)

### Target executable
ament_auto_add_library(pandar_monitor SHARED
  src/pandar_monitor.cpp
)

rclcpp_components_register_node(pandar_monitor
  PLUGIN "pandar_monitor::PandarMonitor"
  EXECUTABLE pandar_monitor_node
)

## Specify libraries to link a library or executable target against
# target_link_libraries(pandar_monitor cpprest crypto fmt)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
  config
)
