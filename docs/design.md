<!-- ## 📜 **`docs/design.md`** -->
# System Architecture & Design

## 🎛️ Controller Module
- Computes gimbal angles for **precise geo-targeting**.
- Supports **PID, LQR, and predictive control** strategies.

## 🎯 System Identification
- Uses **ArUco markers** for real-world calibration.
- Updates **Denavit-Hartenberg (D-H) parameters**.

## 🔗 Interface with UAV (DJI Payload SDK)
- Communicates using **psdk_ros2**.
- Publishes/receives UAV telemetry and control commands.

## 📂 Code Structure

## Package Layout

```plaintext
geogimbal/
├── geogimbal/
│   ├── __init__.py                      # Indicates that this directory is a Python package
│   ├── controller.py                    # Abstract base class for controllers
│   ├── pid_controller.py                # PID controller implementation
│   ├── lqr_controller.py                # LQR controller implementation
│   ├── system_identification.py         # System identification and calibration logic
│   ├── transformations.py               # Coordinate transformation utilities
│   ├── psdk_interface.py                # Interface for psdk_ros2 communication
│   ├── parameters.py                    # Parameter handling
├── scripts/
│   ├── geo_gimbal_control.py            # Main script for gimbal control
│   ├── calibration_node.py              # Script for calibration routines
│   ├── data_analysis.py                 # Scripts for analyzing logged data
│   ├── calibration_tool.py              # Tool for calibrating and updating parameters
├── launch/
│   ├── geo_gimbal_control.launch.py     # Launch file for gimbal control node
│   ├── calibration.launch.py            # Launch file for calibration node
├── config/
│   ├── parameters.yaml                  # YAML file for configurable parameters
│   ├── calibration_parameters.yaml      # Parameters specific to calibration
├── data/
│   ├── calibration_data/                # Stored calibration datasets
│   ├── logs/                            # Data logs from tests and runs
├── setup.py                             # Build instructions for Python package
├── package.xml                          # ROS 2 package metadata
├── README.md                            # Project documentation

```
<!-- 
geogimbal/
├── geogimbal/                  # Main Package Directory
│   ├── controllers/            # Gimbal Controllers (C++)
│   │   ├── base_controller.hpp
│   │   ├── base_controller.cpp
│   │   ├── pid_controller.hpp
│   │   ├── pid_controller.cpp
│   │   ├── lqr_controller.hpp
│   │   ├── lqr_controller.cpp
│   │   ├── predictive_controller.hpp
│   │   ├── predictive_controller.cpp
│   │   ├── factory.hpp          # Factory Pattern to Choose Controller
│   │
│   ├── uav_interfaces/         # UAV Communication Abstraction Layer (C++)
│   │   ├── base_uav_interface.hpp
│   │   ├── base_uav_interface.cpp
│   │   ├── dji_psdk_interface.hpp
│   │   ├── dji_psdk_interface.cpp
│   │   ├── px4_interface.hpp     # Future support for PX4
│   │   ├── px4_interface.cpp
│   │   ├── ardupilot_interface.hpp  # Future support for ArduPilot
│   │   ├── ardupilot_interface.cpp
│   │
│   ├── system_identification/  # Calibration and system modeling (Python)
│   │   ├── __init__.py
│   │   ├── aruco_calibration.py
│   │   ├── transformation_utils.py
│   │   ├── dh_parameter_estimation.py
│   │
│   ├── nodes/                  # ROS 2 Nodes
│   │   ├── gimbal_control_node.cpp  # Main C++ node for real-time control
│   │   ├── calibration_node.py      # Python-based calibration
│   │   ├── mock_publisher_node.py   # Mock ROS publisher (Python)
│   │   ├── mock_service_node.py     # Mock ROS service (Python)
│   │
│   ├── utils/                  # Utility Scripts
│   │   ├── logging_util.py
│   │   ├── ros_param_loader.py
│   │   ├── test_data_generator.py  # Generates mock test data
│   │
├── scripts/                     # Python-based scripts
│   ├── run_gimbal_control.py
│   ├── calibrate_gimbal.py
│
├── config/                      # YAML Configuration Files
│   ├── parameters.yaml          # Controller & UAV params
│   ├── calibration.yaml         # Calibration parameters
│   ├── test_parameters.yaml     # Parameters for testing & mock setup
│
├── launch/                      # Launch Files
│   ├── gimbal_control.launch.py
│   ├── calibration.launch.py
│   ├── mock_test.launch.py       # Launches mock test environment
│
├── tests/                       # Unit and Integration Tests
│   ├── unit_tests/              # Unit Tests (GTest for C++, PyTest for Python)
│   │   ├── test_controllers.cpp
│   │   ├── test_uav_interfaces.cpp
│   │   ├── test_system_identification.py
│   │
│   ├── integration_tests/       # Integration Tests with Mock Data
│   │   ├── test_mock_gimbal_control.py
│   │   ├── test_mock_uav_interface.py
│   │   ├── test_ros_message_passing.py
│
├── docs/                        # Documentation
│   ├── README.md
│   ├── project_overview.md
│   ├── api_reference.md
│   ├── test_plan.md             # Detailed test plan and procedures
│
├── setup.py                     # ROS 2 Python Package Setup
├── package.xml                  # ROS 2 Metadata
├── CMakeLists.txt               # C++ and Python build system -->
