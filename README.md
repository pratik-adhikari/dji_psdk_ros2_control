# GeoGimbal

GeoGimbal is a ROS 2 package intended to control the gimbal of a UAV, enabling the capture of geo-specific images (For implementation, DJI Matrice 300 and DJI PayloadSDK being used). The project focuses on creating a modular structure that separates system identification & control logic for gimbal (from DJI-specific communication), preferabley to make it adaptable for other UAV platforms equipped with reliable positioning sensors like RTK-GNSS.

(This project assumes the gimbal control a solved problem, and targets to model the system for RTK-GNSS to calculate the instantanious gimbal orientation)

Capturing high-resolution images that are precisely aligned with specific geographic coordinates is crucial for applications such as environmental monitoring, infrastructure inspection, and scientific data collection. GeoGimbal provides a robust solution for controlling a UAV's gimbal and camera to achieve this precision.


---
<!-- 
## Contents
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
6. [Parameters and Configuration](#parameters-and-configuration) -->


## 📌 Table of Contents
- [Project Overview](docs/project_overview.md)
- [Installation Guide](docs/installation.md)
- [Usage Instructions](docs/usage.md)
- [System Architecture & Design](docs/design.md)
<!-- - [Troubleshooting](docs/troubleshooting.md) -->

---
<!-- 
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
- Prepare for deployment and maintenance. -->

