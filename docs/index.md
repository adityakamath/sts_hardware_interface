---
layout: page
title: STS Hardware Interface
subtitle: ros2_control SystemInterface for Feetech STS series servo motors
---

<style>
  .feature-box {
    transition: all 0.2s ease;
  }

  .feature-box:hover {
    transform: translateY(-2px);
    box-shadow: 0 4px 12px rgba(0,0,0,0.2) !important;
  }
</style>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;" markdown="1">

![Project Status](https://img.shields.io/badge/Status-WIP-yellow)
![ROS 2](https://img.shields.io/badge/ROS%202-Kilted%20(Ubuntu%2024.04)-blue?style=flat&logo=ros&logoSize=auto)
![ROS 2 Control](https://img.shields.io/badge/ros2__control-SystemInterface-blue?style=flat&logo=ros&logoSize=auto)
![Repository](https://img.shields.io/badge/Repository-adityakamath%2Fsts__hardware__interface-blue?style=flat&logo=github&logoSize=auto)
![Repository](https://img.shields.io/badge/Dependency-adityakamath%2FSCServo__Linux-blue?style=flat&logo=github&logoSize=auto)
![License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface?label=License)

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">

> `ros2_control` SystemInterface for Feetech STS series servo motors (STS3215 and compatible).

**⚠️ Status:** Only **Mode 1 (Velocity)** has been tested. Modes 0 (Position) and 2 (PWM) are experimental.

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">
<div style="display: flex; flex-wrap: wrap; gap: 0.6em; margin: 2em 0; align-items: stretch;">
  <!-- Combined container for Scalable Multi-Motor Support and Multi-Motor Coordination -->
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🚀</span>
    <div style="flex: 1;">
      <strong>Scalable Multi-Motor Coordination</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">
        Control an entire bus simultaneously and synchronize multiple motors with a single command.
      </span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🎛️</span>
    <div style="flex: 1;">
      <strong>Three Operating Modes</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Supports Position (Mode 0), Velocity (Mode 1), and PWM (Mode 2) control modes for each motor.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🔀</span>
    <div style="flex: 1;">
      <strong>Mixed-Mode Operation</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Control different motors in different modes on the same serial bus.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🛡️</span>
    <div style="flex: 1;">
      <strong>Safety Features</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Broadcast emergency stop, enable hardware limits, automatic error recovery.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">📊</span>
    <div style="flex: 1;">
      <strong>Full State Feedback</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Measure position, velocity, load, voltage, temperature, current, motion status.</span>
    </div>
  </div>
  <div class="feature-box" style="flex: 1 1 calc(33.333% - 0.4em); min-width: 280px; display: flex; align-items: flex-start; gap: 0.8em; background: #ffffff; padding: 0.7em 0.9em; border-radius: 8px; box-shadow: 0 4px 12px rgba(0,0,0,0.2);">
    <span style="font-size: 1.8em;">🧪</span>
    <div style="flex: 1;">
      <strong>Mock Mode</strong>
      <br/><br/>
      <span style="font-size: 0.95em;">Control mock interfaces without any hardware for development and testing.</span>
    </div>
  </div>
</div>
</div>

---

## Interfaces

<table class="param-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
      <table class="param-table" style="width: auto; min-width: 340px; max-width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
    <tr>
      <th colspan="4" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🎮 Command Interfaces</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Interface</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Range</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Modes</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>position</code></td>
        <td style="padding: 0.6em; border: none;">0 to 2π radians</td>
        <td style="padding: 0.6em; border: none;">0</td>
        <td style="padding: 0.6em; border: none;">Target joint position (wraps at 2π)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>velocity</code></td>
      <td style="padding: 0.6em; border: none;">-5.22 to +5.22 rad/s (±3400 Steps/s)</td>
        <td style="padding: 0.6em; border: none;">0, 1</td>
      <td style="padding: 0.6em; border: none;">Max speed in Mode 0, Target velocity in Mode 1</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>acceleration</code></td>
      <td style="padding: 0.6em; border: none;">0-254 (unitless)</td>
        <td style="padding: 0.6em; border: none;">0, 1</td>
      <td style="padding: 0.6em; border: none;">Acceleration limit</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>effort</code></td>
      <td style="padding: 0.6em; border: none;">-1.0 to +1.0</td>
        <td style="padding: 0.6em; border: none;">2</td>
      <td style="padding: 0.6em; border: none;">PWM duty cycle</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>emergency_stop</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;">All</td>
      <td style="padding: 0.6em; border: none;">Stops all motors immediately (broadcast)</td>
    </tr>
  </tbody>
</table>

<table class="param-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
      <table class="param-table" style="width: auto; min-width: 340px; max-width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
    <tr>
      <th colspan="3" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🧩 State Interfaces</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Interface</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Unit</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>position</code></td>
      <td style="padding: 0.6em; border: none;">radians</td>
      <td style="padding: 0.6em; border: none;">Current joint position</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>velocity</code></td>
      <td style="padding: 0.6em; border: none;">rad/s</td>
      <td style="padding: 0.6em; border: none;">Current joint velocity</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>effort</code></td>
      <td style="padding: 0.6em; border: none;">%</td>
      <td style="padding: 0.6em; border: none;">Motor load percentage (-100.0 to +100.0)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>voltage</code></td>
      <td style="padding: 0.6em; border: none;">volts</td>
      <td style="padding: 0.6em; border: none;">Supply voltage</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>temperature</code></td>
      <td style="padding: 0.6em; border: none;">°C</td>
      <td style="padding: 0.6em; border: none;">Internal temperature</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>current</code></td>
      <td style="padding: 0.6em; border: none;">amperes</td>
      <td style="padding: 0.6em; border: none;">Motor current draw</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>is_moving</code></td>
      <td style="padding: 0.6em; border: none;">1.0/0.0</td>
      <td style="padding: 0.6em; border: none;">Motion status (1.0 = moving, 0.0 = stopped)</td>
    </tr>
  </tbody>
</table>

---

## Quick Start

### Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/adityakamath/sts_hardware_interface.git
cd sts_hardware_interface
git submodule update --init --recursive
cd ~/ros2_ws
colcon build --packages-select sts_hardware_interface
```

### Basic Configuration

```xml
<ros2_control name="sts_system" type="system">
  <hardware>
    <plugin>sts_hardware_interface/STSHardwareInterface</plugin>
    <param name="serial_port">/dev/ttyACM0</param>
    <param name="baud_rate">1000000</param>
    <param name="use_sync_write">true</param>
  </hardware>

  <joint name="wheel_joint">
    <param name="motor_id">1</param>
    <param name="operating_mode">1</param>

    <command_interface name="velocity"/>
    <command_interface name="acceleration"/>

    <!-- State interfaces (optional declarations for documentation) -->
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <state_interface name="temperature"/>
  </joint>
</ros2_control>
```

---

## Documentation

<div style="display: flex; gap: 1em; margin: 2em 0;">
  <a href="quick-start" style="flex: 1; text-decoration: none;">
    <div style="background: #0366d6; color: white; padding: 1.5em; border-radius: 8px; text-align: center;">
      <h3 style="margin: 0; color: white;">📚 Quick Start Guide</h3>
      <p style="margin: 0.5em 0 0 0; opacity: 0.9;">Setup and usage instructions</p>
    </div>
  </a>
  <a href="architecture" style="flex: 1; text-decoration: none;">
    <div style="background: #28a745; color: white; padding: 1.5em; border-radius: 8px; text-align: center;">
      <h3 style="margin: 0; color: white;">🏗️ Architecture</h3>
      <p style="margin: 0.5em 0 0 0; opacity: 0.9;">Implementation details and design</p>
    </div>
  </a>
</div>

---

## Parameters

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">

<style>
  .param-table {
    transition: all 0.2s ease;
  }

  .param-table:hover {
    transform: translateY(-2px);
    box-shadow: 0 6px 16px rgba(0,0,0,0.25) !important;
  }
</style>

<table class="param-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="4" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">⚙️  Hardware Parameters</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Parameter</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Type</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Default</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>serial_port</code></td>
      <td style="padding: 0.6em; border: none;">string</td>
      <td style="padding: 0.6em; border: none;"><em>required</em></td>
      <td style="padding: 0.6em; border: none;">Serial port path (e.g., <code>/dev/ttyACM0</code>)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>baud_rate</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;">1000000</td>
      <td style="padding: 0.6em; border: none;">Baud rate: 9600-1000000</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>communication_timeout_ms</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;">100</td>
      <td style="padding: 0.6em; border: none;">Serial timeout: 1-1000 ms</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>use_sync_write</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;">true</td>
      <td style="padding: 0.6em; border: none;">Enable SyncWrite for multi-motor setups</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>enable_mock_mode</code></td>
      <td style="padding: 0.6em; border: none;">bool</td>
      <td style="padding: 0.6em; border: none;">false</td>
      <td style="padding: 0.6em; border: none;">Hardware-free simulation mode</td>
    </tr>
  </tbody>
</table>

<table class="param-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="4" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🔧  Joint Parameters</th>
    </tr>
    <tr>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Parameter</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Type</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Default</th>
      <th style="text-align: left; padding: 0.6em; background: #e9ecef; border: none;">Description</th>
    </tr>
  </thead>
  <tbody>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>motor_id</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;"><em>required</em></td>
      <td style="padding: 0.6em; border: none;">Motor ID on serial bus (1-253)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>operating_mode</code></td>
      <td style="padding: 0.6em; border: none;">int</td>
      <td style="padding: 0.6em; border: none;">1</td>
      <td style="padding: 0.6em; border: none;">0=Position, 1=Velocity, 2=PWM</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>min_position</code></td>
      <td style="padding: 0.6em; border: none;">double</td>
      <td style="padding: 0.6em; border: none;">0 radians</td>
      <td style="padding: 0.6em; border: none;">Min position limit (Mode 0 only)</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>max_position</code></td>
      <td style="padding: 0.6em; border: none;">double</td>
      <td style="padding: 0.6em; border: none;">2π radians</td>
      <td style="padding: 0.6em; border: none;">Max position limit (Mode 0 only)</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>max_effort</code></td>
      <td style="padding: 0.6em; border: none;">double</td>
      <td style="padding: 0.6em; border: none;">1.0</td>
      <td style="padding: 0.6em; border: none;">Max PWM duty cycle (0.0-1.0, Mode 2 only)</td>
    </tr>
  </tbody>
</table>

</div>
