cmake_minimum_required(VERSION 3.5)
project(geogimbal)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(psdk_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rmw REQUIRED)
find_package(rosidl_typesupport_interface REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

find_package(yaml-cpp REQUIRED)


# Use pkg-config to find PROJ
find_package(PkgConfig REQUIRED)
pkg_check_modules(PROJ REQUIRED proj)

if (NOT PROJ_FOUND)
  message(FATAL_ERROR "PROJ library not found. Please ensure it is installed.")
endif()

include_directories(
  ${PROJ_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
link_directories(${PROJ_LIBRARY_DIRS})


rosidl_generate_interfaces(${PROJECT_NAME}
  srv/SetTarget.srv
  # If you have messages or other services, list them here
)

# -----------------------------------------------------------
# 1. LIBRARY for the main gimbal logic
# -----------------------------------------------------------
add_library(geogimbal_lib SHARED
  src/coordinate_transformer.cpp
  src/geo_gimbal_control.cpp
  src/gimbal_transform_node.cpp
)

target_include_directories(geogimbal_lib
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(geogimbal_lib
  rclcpp
  sensor_msgs
  std_msgs
  psdk_interfaces
  geometry_msgs
  rmw
  rosidl_typesupport_interface
  rosidl_runtime_c
  tf2
  tf2_geometry_msgs
)

# Include dirs for your library
target_include_directories(geogimbal_lib
  PUBLIC
    include
    ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(geogimbal_lib
  ${PROJ_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  yaml-cpp
)
# so that references to geogimbal::srv::SetTarget compile
ament_export_dependencies(rosidl_default_runtime)
# -----------------------------------------------------------
# 2. Main NODE EXECUTABLE, linking the library
# -----------------------------------------------------------
add_executable(geo_gimbal_main
  src/main.cpp
)

target_include_directories(geo_gimbal_main
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

target_link_libraries(geo_gimbal_main
  geogimbal_lib
  yaml-cpp
  ${PROJ_LIBRARIES}
)

# This line ensures that geo_gimbal_main has the same ament dependencies as geogimbal_lib
ament_target_dependencies(geo_gimbal_main
  rclcpp
  sensor_msgs
  std_msgs
  psdk_interfaces
  geometry_msgs
  rmw
  rosidl_typesupport_interface
  rosidl_runtime_c
)

# -----------------------------------------------------------
# Install the executables
# -----------------------------------------------------------
install(TARGETS
  geo_gimbal_main
  geogimbal_lib
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch/config/data
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(DIRECTORY data DESTINATION share/${PROJECT_NAME})

# -----------------------------------------------------------
# 3. Unit tests
# -----------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)    #for unit_tests using google test
  find_package(ament_cmake_pytest REQUIRED)   #for integration_tests using python
  find_package(launch_testing_ament_cmake REQUIRED)  # For advanced launch testing with .launch.py
  
  # 3.1 GTest for GeoGimbalControl #(unit tests)
  # Create test executable
  ament_add_gtest(
    test_geo_gimbal_control
    tests/unit_tests/test_geo_gimbal_control.cpp
  )
  # Link test to the same library
  target_link_libraries(test_geo_gimbal_control geogimbal_lib yaml-cpp ${PROJ_LIBRARIES})

  ament_target_dependencies(test_geo_gimbal_control
    rclcpp
    sensor_msgs
    std_msgs
    psdk_interfaces
    geometry_msgs
    rmw
    rosidl_typesupport_interface
    rosidl_runtime_c
  )

  # 3.2 GTest for CoordinateTransformer (Optional separate test)
  #     If you have a separate file test_coordinate_transformer.cpp:
  ament_add_gtest(test_coordinate_transformer
    tests/unit_tests/test_coordinate_transformer.cpp
  )
  target_link_libraries(test_coordinate_transformer
    geogimbal_lib
    ${PROJ_LIBRARIES}
  )
  ament_target_dependencies(test_coordinate_transformer
    rclcpp
    sensor_msgs
    std_msgs
    psdk_interfaces
    geometry_msgs
    rmw
    rosidl_typesupport_interface
    rosidl_runtime_c
  )


  # 3.3 Launch test (integration)
  ament_add_pytest_test(
    test_geo_gimbal_launch
    tests/integration_tests/test_geo_gimbal_launch.py
    TIMEOUT 15
  )

endif()

ament_package()