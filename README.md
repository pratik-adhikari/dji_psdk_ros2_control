# GeoGimbal

GeoGimbal is a ROS 2 package designed to precisely control the gimbal and camera of a DJI UAV, enabling the capture of geo-specific images. The project focuses on creating a modular structure that separates gimbal control logic and system identification from DJI-specific communication, making it adaptable for other UAV platforms equipped with reliable positioning sensors like RTK-GNSS.

---

## Table of Contents

1. [Introduction](#introduction)
2. [Project Overview](#project-overview)
   - [Why Geo-Specific Images?](#why-geo-specific-images)
   - [Project Goals](#project-goals)
   - [Key Challenges](#key-challenges)
3. [System Architecture](#system-architecture)
   - [Controller](#controller)
   - [System Identification](#system-identification)
   - [Interface](#interface)
4. [Project Phases and Completion Plan](#project-phases-and-completion-plan)
   - [Phase 1: Initial Analytical Model and Proof of Concept](#phase-1-initial-analytical-model-and-proof-of-concept)
   - [Phase 2: System Identification and Calibration](#phase-2-system-identification-and-calibration)
   - [Phase 3: Controller Design and Implementation](#phase-3-controller-design-and-implementation)
   - [Phase 4: Integration and Comprehensive Testing](#phase-4-integration-and-comprehensive-testing)
   - [Phase 5: Documentation and Deployment](#phase-5-documentation-and-deployment)
5. [Code Structure](#code-structure)
6. [Parameters and Configuration](#parameters-and-configuration)

---

## Introduction

Capturing high-resolution images that are precisely aligned with specific geographic coordinates is crucial for applications such as environmental monitoring, infrastructure inspection, and scientific data collection. GeoGimbal provides a robust solution for controlling a UAV's gimbal and camera to achieve this precision.

---

## Project Overview

### Why Geo-Specific Images?

Geo-specific images are essential when:
- **Precision Matters:** Tasks require data from exact geographic locations.
- **Remote Monitoring:** Access to the area is limited or hazardous.
- **Data Consistency:** Long-term projects need consistent data over time.

### Project Goals
- **Precise Gimbal Control:** Orient the UAV's gimbal towards specified RTK GNSS coordinates accurately.
- **Modular Architecture:** Separate control logic from hardware-specific implementations for adaptability.
- **System Identification:** Develop mathematical models to improve system accuracy through calibration.
- **Robust Controller Design:** Implement controllers to handle system dynamics and sensor limitations.

### Key Challenges
- **Accurate Transformation:** Precise transformations accounting for offsets and dynamics.
- **System Dynamics:** Managing UAV movement, latency, and sensor inaccuracies.
- **Modularity:** Adapting the system for different hardware platforms.

---

## System Architecture

GeoGimbal consists of three primary components:

### Controller
- **Function:** Calculates gimbal angles to align the camera with target coordinates.
- **Features:**
  - Supports multiple control strategies (PID, LQR, predictive).
  - Operates independently of hardware-specific details.

### System Identification
- **Function:** Calibrates and refines models mapping GNSS positions to gimbal commands.
- **Features:**
  - Uses visual markers (e.g., ArUco) for calibration.
  - Updates Denavit-Hartenberg (D-H) parameters.

### Interface
- **Function:** Communicates with DJI hardware via `psdk_ros2`.
- **Features:**
  - Subscribes to UAV data topics.
  - Publishes gimbal commands.
  - Acts as a bridge between logic and hardware.

---

## Project Phases and Completion Plan

### Phase 1: Initial Analytical Model and Proof of Concept
- Develop transformation matrices.
- Implement geodetic conversions.
- Test basic functionality.

### Phase 2: System Identification and Calibration
- Use ArUco markers for calibration.
- Refine models using error analysis.

### Phase 3: Controller Design and Implementation
- Implement controllers (PID, LQR, predictive).
- Optimize for system dynamics and latency.

### Phase 4: Integration and Comprehensive Testing
- Integrate components.
- Conduct real-world tests and optimize.

### Phase 5: Documentation and Deployment
- Finalize documentation.
- Prepare for deployment and maintenance.

---
# 5. Code Structure

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
---

# 6. Parameters and Configuration
## Parameters to be Saved

- **Transformation Matrices**:
  - Offsets and rotations from RTK-GNSS sensor to UAV center.
  - From UAV center through the gimbal to the camera.
- **D-H Parameters**:
  - Kinematic chain parameters used in the model.
- **Calibration Data**:
  - Errors observed during system identification.
  - Updated parameters after calibration.
- **Controller Parameters**:
  - Gains for PID controllers.
  - Weight matrices for LQR controllers.
  - Prediction horizons for predictive controllers.
- **Sensor Data Logs**:
  - RTK-GNSS readings.
  - UAV pose and orientation.
  - Gimbal joint angles.

## Adjusting Parameters

- Configurable via `config/parameters.yaml`.
- Use for:
  - Fine-tuning controller performance.
  - Updating system models after calibration.
  - Adjusting for different hardware configurations.
