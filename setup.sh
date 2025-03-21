#!/bin/bash

# Initialize git repository
git init

# Create package directories with ROS2 structure
for pkg in geogimbal mission_bt waypoint_manager camera_controller tag_tracker psdk_interface; do
    mkdir -p $pkg/{src,include/$pkg,config,launch,msg,srv,test}
    
    # Create package.xml template
    cat > $pkg/package.xml << EOL
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>$pkg</name>
  <version>0.1.0</version>
  <description>$pkg package for DJI drone control</description>
  <maintainer email="your.email@example.com">Your Name</maintainer>
  <license>MIT</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
EOL

    # Create CMakeLists.txt template
    cat > $pkg/CMakeLists.txt << EOL
cmake_minimum_required(VERSION 3.8)
project(${pkg})

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
EOL

    # Create README
    cat > $pkg/README.md << EOL
# ${pkg}

ROS2 package for DJI drone control system.
EOL
done

# Create docs
mkdir -p docs
cat > docs/README.md << EOL
# DJI PSDK ROS2 Drone Control Documentation

## Overview
This repository contains modular ROS2 packages for DJI drone control using DJI PSDK.

## Package Details
- geogimbal: Geographic to gimbal angle computations
- mission_bt: Behavior Tree-based mission orchestration
- waypoint_manager: Waypoint navigation and path management
- camera_controller: Camera operations for DJI payloads
- tag_tracker: AprilTag-based alignment and tracking
- psdk_interface: PSDK ROS2 adapter interface

[Full documentation to be added]
EOL

# Create root README
cat > README.md << EOL
# DJI PSDK ROS2 Drone Control

Modular ROS2-based architecture for DJI drone control using PSDK.

## Packages
- geogimbal: Geographic/gimbal computations
- mission_bt: Behavior Tree orchestration
- waypoint_manager: Navigation control
- camera_controller: Camera operations
- tag_tracker: Visual tracking
- psdk_interface: PSDK adapter

## Documentation
See [docs/](docs/README.md) for full documentation.

## License
MIT License
EOL

# Create LICENSE
cat > LICENSE << EOL
MIT License

Copyright (c) 2024 Your Name

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
EOL

# Initial git commit
git add .
git commit -m "Initial commit: Setup modular ROS2 package structure for DJI drone control"

# Add remote and push (uncomment and modify as needed)
# git remote add origin https://github.com/pratik-adhikari/dji_psdk_ros2_control.git
# git push -u origin main
