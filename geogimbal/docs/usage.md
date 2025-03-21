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