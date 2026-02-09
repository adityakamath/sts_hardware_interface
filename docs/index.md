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
![Repository](https://img.shields.io/badge/Repo-adityakamath%2Fsts__hardware__interface-purple?style=flat&logo=github&logoSize=auto)
![Repository](https://img.shields.io/badge/Dep-adityakamath%2FSCServo__Linux-purple?style=flat&logo=github&logoSize=auto)
[![Blog](https://img.shields.io/badge/Blog-kamathrobotics.com-darkorange?style=flat&logo=hashnode&logoSize=auto)](https://kamathrobotics.com/hardware-abstraction-for-sts3215-servos)
[![Ask DeepWiki (Experimental)](https://deepwiki.com/badge.svg)](https://deepwiki.com/adityakamath/sts_hardware_interface)
![C++](https://img.shields.io/badge/C++-17-blue?style=flat&logo=cplusplus&logoColor=white)
![License](https://img.shields.io/github/license/adityakamath/sts_hardware_interface?label=License)

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;" markdown="1">

> `ros2_control` `SystemInterface` for Feetech STS series servo motors (STS3215 and compatible).
>
> **⚠️ Status:** **Modes 0 (Position) and 1 (Velocity)** have been tested and validated. Mode 2 (PWM) is implemented but currently untested.

</div>

<div style="width: 100vw; margin-left: calc(-50vw + 50%); padding-left: 5%; padding-right: 5%; box-sizing: border-box;">
<div style="display: flex; flex-wrap: wrap; gap: 0.6em; margin: 2em 0; align-items: stretch;">
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
      <td style="padding: 0.6em; border: none;">-5.22 to +5.22 rad/s (±3400 Steps/s for STS3215)</td>
        <td style="padding: 0.6em; border: none;">0, 1</td>
      <td style="padding: 0.6em; border: none;">Max speed in Mode 0, Target velocity in Mode 1. Range depends on motor model (configurable via max_velocity_steps)</td>
    </tr>
    <tr style="background: #ffffff;">
      <td style="padding: 0.6em; border: none;"><code>acceleration</code></td>
      <td style="padding: 0.6em; border: none;">0-254 (unitless)</td>
        <td style="padding: 0.6em; border: none;">0, 1</td>
      <td style="padding: 0.6em; border: none;">Acceleration value in Mode 0 and 1</td>
    </tr>
    <tr style="background: #f0f0f0;">
      <td style="padding: 0.6em; border: none;"><code>effort</code></td>
      <td style="padding: 0.6em; border: none;">-1.0 to +1.0</td>
        <td style="padding: 0.6em; border: none;">2</td>
      <td style="padding: 0.6em; border: none;">PWM duty cycle (open-loop, no velocity/acceleration)</td>
    </tr>
  </tbody>
</table>

<table class="param-table" style="width: 100%; border-collapse: separate; border-spacing: 0; margin: 2em auto; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 12px rgba(0,0,0,0.2); border: none;">
  <thead>
    <tr>
      <th colspan="4" style="text-align: center; padding: 0.6em; background: #f8f9fa; border: none;">🧩 State Interfaces</th>
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
      <td style="padding: 0.6em; border: none;">normalized</td>
      <td style="padding: 0.6em; border: none;">Motor load (-1.0 to +1.0, absolute value)</td>
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

</div>

</div>
