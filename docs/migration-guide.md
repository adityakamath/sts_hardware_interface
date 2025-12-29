# Migration Summary: sts_actuator_interface → sts_hardware_interface

## Date
December 27, 2025

## Overview
Moved the STS servo motor hardware interface from `lekiwi_ros2` package to a new standalone `sts_hardware_interface` package, renaming from "actuator" to "hardware" terminology.

## New Package: sts_hardware_interface

### Created Files
1. **include/sts_hardware_interface/sts_hardware_interface.hpp**
   - Renamed from `STSActuatorInterface` to `STSHardwareInterface`
   - Changed namespace from `lekiwi_ros2` to `sts_hardware_interface`
   - Updated header guards and documentation

2. **src/sts_hardware_interface.cpp**
   - Renamed class from `STSActuatorInterface` to `STSHardwareInterface`
   - Updated logger names from `STSActuatorInterface` to `STSHardwareInterface`
   - Updated comments: "actuator" → "hardware interface"
   - Changed plugin export from `lekiwi_ros2::STSActuatorInterface` to `sts_hardware_interface::STSHardwareInterface`

3. **sts_hardware_interface.xml**
   - Plugin descriptor for ros2_control
   - Defines `sts_hardware_interface/STSHardwareInterface` plugin

4. **CMakeLists.txt**
   - Configured to build the hardware interface library
   - Links against SCServo library from lekiwi_ros2
   - Exports the plugin via pluginlib

5. **package.xml**
   - Package metadata and dependencies
   - Exports hardware_interface plugin
   - Depends on lekiwi_ros2 for SCServo library

6. **README.md**
   - Comprehensive documentation
   - Usage examples
   - Migration guide

## Modified Files in lekiwi_ros2

### 1. urdf/lekiwi_hardware.urdf.xacro
**Changed**: All three motor hardware plugin references

```diff
- <plugin>lekiwi_ros2/STSActuatorInterface</plugin>
+ <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
```

Applied to:
- Left motor (motor_left)
- Back motor (motor_back)
- Right motor (motor_right)

### 2. CMakeLists.txt
**Removed**: `src/sts_actuator_interface.cpp` from library build

```diff
- # STS Actuator Hardware Interface (ros2_control plugin)
+ # Lekiwi System Hardware Interface (ros2_control plugin)
+ # Note: Individual motor control moved to sts_hardware_interface package
  add_library(lekiwi_ros2_hardware_interface SHARED
-   src/sts_actuator_interface.cpp
    src/lekiwi_system_interface.cpp
  )
```

### 3. lekiwi_ros2_hardware_interface.xml
**Removed**: `STSActuatorInterface` class definition
**Added**: Comment noting the migration

```diff
  <?xml version="1.0"?>
  <library path="lekiwi_ros2_hardware_interface">
-   <class
-     name="lekiwi_ros2/STSActuatorInterface"
-     type="lekiwi_ros2::STSActuatorInterface"
-     base_class_type="hardware_interface::ActuatorInterface">
-     <description>...</description>
-   </class>
+   <!-- Individual motor control moved to sts_hardware_interface package -->
    
    <class
      name="lekiwi_ros2/LekiwiSystemInterface"
      type="lekiwi_ros2::LekiwiSystemInterface"
      base_class_type="hardware_interface::SystemInterface">
      <description>...</description>
    </class>
  </library>
```

### 4. package.xml
**Added**: Runtime dependency on sts_hardware_interface

```diff
  <!-- ros2_control dependencies -->
  <depend>hardware_interface</depend>
  <depend>pluginlib</depend>
  <depend>controller_interface</depend>
  <depend>realtime_tools</depend>
+
+ <!-- STS Hardware Interface (for individual motor control) -->
+ <exec_depend>sts_hardware_interface</exec_depend>
```

## Files to Keep in lekiwi_ros2 (Not Deleted)

The following files remain in lekiwi_ros2 but are now obsolete:
- `include/lekiwi_ros2/sts_actuator_interface.hpp`
- `src/sts_actuator_interface.cpp`

**Recommendation**: These can be deleted after confirming the new package builds and works correctly.

## Next Steps

1. **Build the new package:**
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select sts_hardware_interface
   ```

2. **Rebuild lekiwi_ros2:**
   ```bash
   colcon build --packages-select lekiwi_ros2
   ```

3. **Test the hardware interface:**
   - Launch the robot with the updated URDF
   - Verify motors are recognized and controllable
   - Check that all state interfaces report correctly

4. **Optional - Clean up lekiwi_ros2:**
   After confirming everything works:
   - Delete `include/lekiwi_ros2/sts_actuator_interface.hpp`
   - Delete `src/sts_actuator_interface.cpp`
   - Update documentation references

## Benefits of This Migration

1. **Reusability**: sts_hardware_interface can now be used in any ROS2 project
2. **Separation of Concerns**: Motor interface is independent of robot-specific code
3. **Better Organization**: Clear distinction between individual motor control and system-level control
4. **Easier Maintenance**: Changes to motor interface don't require rebuilding entire lekiwi package

## Breaking Changes

Users of the lekiwi_ros2 package who were referencing `lekiwi_ros2/STSActuatorInterface` in their URDFs must:
1. Install the new `sts_hardware_interface` package
2. Update URDF plugin references as shown above

The `lekiwi_ros2/LekiwiSystemInterface` is unchanged and continues to work as before.
