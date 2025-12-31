# Performance Monitoring Guide

The STS Hardware Interface does not include built-in performance monitoring code. Instead, use ros2_control's standard tooling and ROS 2 diagnostic capabilities for performance analysis.

## Why No Built-In Monitoring?

**Design Decision**: Built-in performance monitoring was removed to:
1. Reduce code complexity and maintenance burden
2. Avoid runtime overhead in production systems
3. Leverage ros2_control's standard monitoring infrastructure
4. Keep the hardware interface focused on hardware communication

## Recommended Monitoring Approaches

### 1. ros2_control Controller Manager Statistics

The Controller Manager provides built-in performance metrics for all hardware interfaces.

#### Enable Statistics

In your controller manager configuration:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50  # Hz

    # Enable real-time statistics
    use_sim_time: false
    enable_statistics: true
```

#### View Statistics

```bash
# View controller manager diagnostics
ros2 topic echo /controller_manager/diagnostics

# View update rate and timing
ros2 topic echo /controller_manager/statistics
```

The statistics topic publishes:
- Actual update rate
- Maximum cycle time
- Minimum cycle time
- Average cycle time
- Missed deadlines

### 2. ROS 2 Performance Tools

#### Using `ros2 topic hz`

Monitor update rates of state publishers:

```bash
# Monitor joint state publication rate
ros2 topic hz /joint_states

# Expected output: ~50 Hz for LeKiwi (controller update_rate)
```

#### Using `ros2 topic bw`

Monitor bandwidth usage:

```bash
# Monitor joint state bandwidth
ros2 topic bw /joint_states

# Monitor odometry bandwidth
ros2 topic bw /odom
```

### 3. Real-Time Performance Analysis

#### Using `ros2_tracing`

For detailed real-time performance analysis:

```bash
# Install ros2_tracing
sudo apt install ros-${ROS_DISTRO}-tracetools

# Start tracing
ros2 trace start session_name

# Run your robot
ros2 launch lekiwi_base_control lekiwi_base.launch.py

# Stop tracing
ros2 trace stop session_name

# Analyze with babeltrace2
babeltrace2 ~/.ros/tracing/session_name
```

#### Using `cyclonedds_cli`

If using CycloneDDS (default in ROS 2 Humble+):

```bash
# Install CycloneDDS tools
sudo apt install ros-${ROS_DISTRO}-cyclonedds-tools

# Monitor DDS performance
ros2 run cyclonedds_tools dds_profiling
```

### 4. System-Level Monitoring

#### CPU Usage

```bash
# Monitor CPU usage by process
top -p $(pgrep -f controller_manager)

# Or use htop for better visualization
htop -p $(pgrep -f controller_manager)
```

#### Real-Time Scheduling

Check if controller_manager is running with real-time priority:

```bash
# Check scheduling policy and priority
chrt -p $(pgrep -f controller_manager)

# Should show:
# scheduling policy: SCHED_FIFO or SCHED_RR
# scheduling priority: 50-90 (higher = more priority)
```

### 5. Custom Diagnostics (Optional)

If you need custom performance metrics, use ROS 2 diagnostic_updater:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class JointStateMonitor(Node):
    def __init__(self):
        super().__init__('joint_state_monitor')
        self.last_time = None
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10)

    def callback(self, msg):
        now = time.time()
        if self.last_time is not None:
            period = now - self.last_time
            rate = 1.0 / period if period > 0 else 0.0
            self.get_logger().info(f'Joint state rate: {rate:.2f} Hz')
        self.last_time = now

def main():
    rclpy.init()
    monitor = JointStateMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 6. Logging for Debugging

For one-time debugging, add temporary logging to controller_manager:

```bash
# Enable DEBUG logging for controller_manager
ros2 run controller_manager ros2_control_node \
  --ros-args \
  --log-level controller_manager:=DEBUG
```

This will show:
- Read/write cycle timing
- Hardware interface errors
- Controller state transitions

## Performance Targets

### Expected Performance (LeKiwi Configuration)

| Metric | Target | Notes |
|--------|--------|-------|
| Update Rate | 50 Hz | Configured in controller YAML |
| Read Latency | < 10 ms | Serial + computation |
| Write Latency | < 5 ms | Serial + SyncWrite |
| Total Cycle Time | < 20 ms | Should be well under 1/50 = 20ms |

### Performance Degradation Indicators

Watch for these warning signs:

1. **Missed Deadlines**: Controller manager reports missed cycles
2. **High Jitter**: Inconsistent update rates (use `ros2 topic hz`)
3. **Serial Timeouts**: RCLCPP_WARN messages about FeedBack failures
4. **CPU Spikes**: Controller_manager using >50% of single core

## Troubleshooting Performance Issues

### Issue: Update rate below target (e.g., 30 Hz instead of 50 Hz)

**Possible causes**:
1. Serial communication slow/unreliable
2. Too many state interfaces enabled
3. CPU overload
4. Non-real-time kernel

**Solutions**:
```bash
# Check serial port speed
stty -F /dev/ttySERVO

# Reduce state interfaces in URDF (remove unused ones)
# Consider disabling: load, voltage, temperature, current, is_moving

# Enable real-time scheduling
sudo chrt -f 50 -p $(pgrep -f controller_manager)
```

### Issue: High cycle time variance

**Possible causes**:
1. System interrupts
2. Other processes interfering
3. Disk I/O blocking

**Solutions**:
```bash
# Isolate CPU cores for real-time tasks
sudo isolcpus=2,3  # In /etc/default/grub

# Disable CPU frequency scaling
sudo cpupower frequency-set -g performance

# Use real-time kernel (optional)
sudo apt install linux-lowlatency
```

### Issue: Increasing consecutive errors

**Check logs**:
```bash
ros2 topic echo /diagnostics | grep -A 10 STSHardwareInterface
```

**Common fixes**:
- Check serial cable quality
- Verify baud rate (should be 1000000)
- Check motor IDs are correct
- Reduce bus load (fewer state interfaces)

## Best Practices

1. **Start with monitoring**: Always monitor before optimizing
2. **Use standard tools**: Prefer ros2_control tools over custom solutions
3. **Profile first**: Identify bottlenecks before making changes
4. **Test iteratively**: Change one thing at a time
5. **Document baselines**: Record normal performance for comparison

## References

- [ros2_control Performance Documentation](https://control.ros.org/master/doc/ros2_control/doc/index.html)
- [ROS 2 QoS Settings](https://docs.ros.org/en/rolling/Concepts/About-Quality-of-Service-Settings.html)
- [Real-Time Programming in ROS 2](https://design.ros2.org/articles/realtime_background.html)
- [ros2_tracing Tutorial](https://github.com/ros2/ros2_tracing)

## Summary

**Bottom line**: The STS Hardware Interface delegates performance monitoring to standard ROS 2 tooling. This keeps the code simple while providing powerful monitoring capabilities through the ros2_control framework and ROS 2 ecosystem tools.

For most applications, monitoring `/controller_manager/statistics` and using `ros2 topic hz /joint_states` is sufficient. For advanced debugging, use ros2_tracing or custom diagnostic nodes.
