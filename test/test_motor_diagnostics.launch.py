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

import launch
import launch_testing
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
from control_msgs.msg import DynamicJointState

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('sts_hardware_interface')
    diagnostics_launch = os.path.join(pkg_share, 'launch', 'motor_diagnostics.launch.py')
    single_motor_launch = os.path.join(pkg_share, 'launch', 'single_motor.launch.py')

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
        received = []
        sub = self.node.create_subscription(DiagnosticArray, '/diagnostics', received.append, 10)
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return received

    def test_diagnostics_published(self):
        """Verify /diagnostics is published after stack starts."""
        received = self._wait_for_diagnostics(timeout_sec=30.0)
        self.assertTrue(received, '/diagnostics not received within 30 seconds')

    def test_diagnostics_ok_status(self):
        """Verify diagnostics status is OK when all values are nominal."""
        received = self._wait_for_diagnostics(timeout_sec=30.0)
        self.assertTrue(received)
        status = received[0].status[0]
        self.assertEqual(status.level, 0, f"Expected OK (0), got {status.level}")

    def _publish_dynamic_joint_state(self, name, effort=None, current=None, voltage=None, temperature=None):
        pub = self.node.create_publisher(DynamicJointState, '/dynamic_joint_states', 10)
        msg = DynamicJointState()
        msg.joint_names = [name]
        if effort is not None:
            msg.interface_values.append(DynamicJointState.InterfaceValue(interface_name='effort', value=effort))
        if current is not None:
            msg.interface_values.append(DynamicJointState.InterfaceValue(interface_name='current', value=current))
        if voltage is not None:
            msg.interface_values.append(DynamicJointState.InterfaceValue(interface_name='voltage', value=voltage))
        if temperature is not None:
            msg.interface_values.append(DynamicJointState.InterfaceValue(interface_name='temperature', value=temperature))
        # Publish several times to ensure delivery
        for _ in range(5):
            pub.publish(msg)
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_publisher(pub)

    def test_diagnostics_warn_temperature(self):
        """Diagnostics status is WARN when temperature exceeds temp_warn but below temp_error."""
        self._publish_dynamic_joint_state('wheel_joint', temperature=65.0)  # temp_warn=60, temp_error=75
        received = self._wait_for_diagnostics(timeout_sec=5.0)
        self.assertTrue(received)
        status = received[-1].status[0]
        self.assertEqual(status.level, 1, f"Expected WARN (1), got {status.level}")
        self.assertIn('temperature', status.message)

    def test_diagnostics_error_temperature(self):
        """Diagnostics status is ERROR when temperature exceeds temp_error."""
        self._publish_dynamic_joint_state('wheel_joint', temperature=80.0)  # temp_error=75
        received = self._wait_for_diagnostics(timeout_sec=5.0)
        self.assertTrue(received)
        status = received[-1].status[0]
        self.assertEqual(status.level, 2, f"Expected ERROR (2), got {status.level}")
        self.assertIn('temperature', status.message)

    def test_diagnostics_warn_voltage(self):
        """Diagnostics status is WARN when voltage below voltage_min."""
        self._publish_dynamic_joint_state('wheel_joint', voltage=5.0)  # voltage_min=6.0
        received = self._wait_for_diagnostics(timeout_sec=5.0)
        self.assertTrue(received)
        status = received[-1].status[0]
        self.assertEqual(status.level, 1, f"Expected WARN (1), got {status.level}")
        self.assertIn('voltage', status.message)

    def test_diagnostics_error_current(self):
        """Diagnostics status is ERROR when current exceeds current_max."""
        self._publish_dynamic_joint_state('wheel_joint', current=4.0)  # current_max=3.0
        received = self._wait_for_diagnostics(timeout_sec=5.0)
        self.assertTrue(received)
        status = received[-1].status[0]
        self.assertEqual(status.level, 2, f"Expected ERROR (2), got {status.level}")
        self.assertIn('current', status.message)
