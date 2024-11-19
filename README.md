# GeoGimbal

**GeoGimbal** (temporary project name) is a ROS 2 package designed to control the gimbal of a DJI UAV and capture geo-specific images using the `psdk_ros2` wrapper. The project focuses on creating a modular structure to separate gimbal control logic and system identification from DJI-specific communication. 

The system's primary goal is to compute and apply precise gimbal angles (pitch and yaw) to align the UAV camera with specific RTK GNSS locations.

---

## **1. Why Geo-Specific Images?**

High-resolution geo-specific images are essential for tasks that require accurate data collection over predefined geographic coordinates. For example:
- Monitoring infrastructure or vegetation.
- Collecting scientific data with precise spatial references.
- Conducting visual inspections of specified areas.

To achieve this, the UAV gimbal must orient the camera toward predefined GNSS coordinates while compensating for UAV movement and positioning errors.

---

## **2. Project Intent**

The project aims to:
1. Control the DJI UAV gimbal to align with specified RTK GNSS coordinates.
2. Separate the control logic and system identification models from hardware-specific implementations.
3. Provide a clean and modular codebase by using an interface class for managing DJI-specific communication via `psdk_ros2`.

### **Key Focus Areas**
1. **System Identification**:
   - Develop models to translate RTK GNSS data to gimbal commands.
   - Implement transformations to map global coordinates to gimbal angles.
   
2. **Controller Design**:
   - Compute gimbal angles (pitch and yaw) based on input parameters (RTK GNSS position, UAV pose, etc.).
   - Isolate the logic from hardware-specific details for better maintainability.

3. **Interface Design**:
   - Manage communication with DJI-specific services and topics provided by the `psdk_ros2` wrapper.
   - Act as a bridge between the control logic and the DJI hardware.

---

## **3. System Architecture**

GeoGimbal is divided into three primary components:

### **1. Controller**
- Implements the core logic to calculate gimbal angles.
- Operates on structured inputs such as target GNSS coordinates and UAV pose.
- Outputs the required gimbal angles (pitch, yaw) without handling communication or ROS-specific tasks.

**Responsibilities**:
- Process inputs (RTK GNSS and UAV pose).
- Perform necessary coordinate transformations.
- Output gimbal commands for the interface.

---

### **2. System Identification**
- Handles the development of the mathematical model to:
  - Map GNSS positions to gimbal angles.
  - Translate UAV-relative coordinates into gimbal-specific commands.
- This is a critical task to ensure accurate and smooth gimbal control.

---

### **3. Interface**
- Manages all communication with the DJI `psdk_ros2` wrapper.
- Subscribes to GNSS and UAV pose topics, and sends commands to the gimbal via the appropriate services or topics.

**Responsibilities**:
- Convert the controller’s outputs into messages compatible with the DJI `psdk_ros2` wrapper.
- Publish gimbal commands to the `/wrapper/psdk_ros2/gimbal_rotation` topic.
- Call relevant services like `/wrapper/psdk_ros2/gimbal_reset` or `/wrapper/psdk_ros2/gimbal_set_mode` as needed.

---

## **4. Topics and Services**

### **Topics**
1. **Subscribed Topics**:
   - `/drone/gnss` (`sensor_msgs/msg/NavSatFix`):
     - Provides the UAV's RTK GNSS data.
   - `/drone/pose` (`geometry_msgs/msg/Pose`):
     - Provides the UAV's current orientation and position.

2. **Published Topics**:
   - `/wrapper/psdk_ros2/gimbal_rotation` (`psdk_interfaces/msg/GimbalRotation`):
     - Sends gimbal control commands.

---

### **Services**
1. `/wrapper/psdk_ros2/gimbal_reset`:
   - Resets the gimbal to its neutral position.

2. `/wrapper/psdk_ros2/gimbal_set_mode`:
   - Sets the gimbal's operational mode.

---

## **5. Code Structure**

### **Package Layout**
```plaintext
geogimbal/
├── include/
│   └── geogimbal/
│       ├── controller.hpp       # Gimbal control logic
│       ├── psdk_interface.hpp   # Interface for psdk_ros2 communication
├── src/
│   ├── controller.cpp           # Implementation of gimbal control logic
│   ├── psdk_interface.cpp       # Implementation of psdk_ros2 communication
│   ├── main.cpp                 # Entry point for initializing nodes
├── launch/
│   └── geogimbal.launch.py      # Launch file for starting all nodes
├── CMakeLists.txt               # Build instructions
├── package.xml                  # ROS 2 package metadata
