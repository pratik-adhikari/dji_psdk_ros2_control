## 📜 **`docs/project_overview.md`**
# Project Overview

## 🎯 Why Geo-Specific Images?
Geo-specific images are useful for:
- **Precision Data Collection**: Ensures accurate geo-referenced images.
- **Remote Monitoring**: Ideal for inaccessible locations.
- **Consistency in Analysis**: Critical for scientific and environmental studies.

## 🎯 Project Goals
- **Accurate Gimbal Control**: Align the UAV’s gimbal with RTK-GNSS coordinates.
- **Modular Software Design**: Independent control logic from hardware-specific communication.
- **System Identification**: Calibrate and refine coordinate transformations.
- **Robust Controllers**: Implement **PID**, **LQR**, and **predictive control**.

## ⚡ Key Challenges
- **Transformation Accuracy**: Converting GNSS data into gimbal commands.
- **UAV Movement & Latency**: Real-time corrections to avoid drift.
- **Hardware Modularity**: Adapting to different UAV gimbal systems.

For technical details, see **[System Architecture](design.md)**.
