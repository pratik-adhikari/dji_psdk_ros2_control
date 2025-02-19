cmake_minimum_required(VERSION 3.5)
project(geogimbal)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(psdk_interfaces REQUIRED)

# Use pkg-config to find PROJ
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)
if (NOT PROJ_FOUND)
  message(FATAL_ERROR "PROJ library not found. Please ensure it is installed.")
endif()

# Include directories for PROJ
include_directories(
  ${PROJ_INCLUDE_DIRS}
)

# Link directories for PROJ
link_directories(${PROJ_LIBRARY_DIRS})

# Add the executable
add_executable(geo_gimbal_control
  src/geo_gimbal_control.cpp
  src/main.cpp
)

# Add include directories for the target
target_include_directories(geo_gimbal_control PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link dependencies
ament_target_dependencies(geo_gimbal_control
  rclcpp
  sensor_msgs
  std_msgs
  psdk_interfaces
)

# Link the PROJ library
target_link_libraries(geo_gimbal_control ${PROJ_LIBRARIES})

# Install targets
install(TARGETS geo_gimbal_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install configuration files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

# Install data files
install(DIRECTORY data
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
