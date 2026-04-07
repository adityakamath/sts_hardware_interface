# Copyright 2026 Aditya Kamath
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Integration tests for the motor_diagnostics_node in mock mode.

Verifies diagnostics are published and reflect joint state conditions.
"""

import os
import time
import unittest

from control_msgs.msg import DynamicJointState, InterfaceValue
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import qos_profile_sensor_data


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('sts_hardware_interface')
    diagnostics_launch = os.path.join(pkg_share, 'launch', 'motor_diagnostics.launch.py')
    single_motor_launch = os.path.join(pkg_share, 'launch', 'single_motor_velocity.launch.py')

    diagnostics = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(diagnostics_launch),
        launch_arguments={}.items()
    )
    single_motor = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_motor_launch),
        launch_arguments={
            'use_mock': 'true',
            'gui': 'false',
        }.items()
    )

    return launch.LaunchDescription([
        single_motor,
        diagnostics,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMotorDiagnosticsIntegration(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_motor_diagnostics_node')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_diagnostics(self, timeout_sec=30.0):
        """Wait for any /diagnostics message."""
        received = []
        sub = self.node.create_subscription(
            DiagnosticArray, '/diagnostics', received.append, qos_profile_sensor_data)
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return received

    def _wait_for_diagnostic_level(
            self, name, expected_level, keyword, timeout_sec=5.0, **values):
        """Publish fault values in a loop until a matching diagnostic status is received."""
        matched = []

        def on_diag(msg):
            for s in msg.status:
                if s.level == expected_level and keyword in s.message:
                    matched.append(s)

        sub = self.node.create_subscription(
            DiagnosticArray, '/diagnostics', on_diag, qos_profile_sensor_data)
        pub = self.node.create_publisher(DynamicJointState, '/dynamic_joint_states', 10)

        msg = DynamicJointState()
        msg.joint_names = [name]
        iv = InterfaceValue()
        for k, v in values.items():
            iv.interface_names.append(k)
            iv.values.append(float(v))
        msg.interface_values.append(iv)

        deadline = time.time() + timeout_sec
        while time.time() < deadline and not matched:
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.node.destroy_publisher(pub)
        self.node.destroy_subscription(sub)
        return matched

    def test_diagnostics_published(self):
        """Verify /diagnostics is published after stack starts."""
        received = self._wait_for_diagnostics(timeout_sec=30.0)
        self.assertTrue(received, '/diagnostics not received within 30 seconds')

    def test_diagnostics_ok_status(self):
        """Verify diagnostics status is OK when all values are nominal."""
        received = self._wait_for_diagnostics(timeout_sec=30.0)
        self.assertTrue(received)
        status = received[0].status[0]
        self.assertEqual(
            status.level, DiagnosticStatus.OK,
            f'Expected OK ({DiagnosticStatus.OK}), got {status.level}')

    def test_diagnostics_warn_temperature(self):
        """Diagnostics status is WARN when temperature exceeds temp_warn but below temp_error."""
        matched = self._wait_for_diagnostic_level(
            'wheel_joint', DiagnosticStatus.WARN, 'temperature',
            temperature=65.0)  # temp_warn=60, temp_error=75
        self.assertTrue(matched, 'Expected WARN diagnostic for temperature')

    def test_diagnostics_error_temperature(self):
        """Diagnostics status is ERROR when temperature exceeds temp_error."""
        matched = self._wait_for_diagnostic_level(
            'wheel_joint', DiagnosticStatus.ERROR, 'temperature',
            temperature=80.0)  # temp_error=75
        self.assertTrue(matched, 'Expected ERROR diagnostic for temperature')

    def test_diagnostics_warn_voltage(self):
        """Diagnostics status is WARN when voltage below voltage_min."""
        matched = self._wait_for_diagnostic_level(
            'wheel_joint', DiagnosticStatus.WARN, 'voltage',
            voltage=5.0)  # voltage_min=6.0
        self.assertTrue(matched, 'Expected WARN diagnostic for voltage')

    def test_diagnostics_warn_current(self):
        """Diagnostics status is WARN when current exceeds current_max."""
        matched = self._wait_for_diagnostic_level(
            'wheel_joint', DiagnosticStatus.WARN, 'current',
            current=4.0)  # current_max=3.0
        self.assertTrue(matched, 'Expected WARN diagnostic for current')
