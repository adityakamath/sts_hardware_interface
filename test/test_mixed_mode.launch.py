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
Launch test for mixed_mode example in mock mode.

Verifies that the complete ros2_control stack starts with multiple motors
in different operating modes (position, velocity, effort) using mock hardware.
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


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Generate launch description for the mixed mode integration test."""
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('sts_hardware_interface')
    mixed_mode_launch = os.path.join(pkg_share, 'launch', 'mixed_mode.launch.py')

    mixed_mode = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mixed_mode_launch),
        launch_arguments={
            'use_mock': 'true',
            'gui': 'false',
        }.items()
    )

    return launch.LaunchDescription([
        mixed_mode,
        launch_testing.actions.ReadyToTest(),
    ])


class TestMixedModeLaunch(unittest.TestCase):
    """Test that mixed mode stack launches with all three controllers active."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mixed_mode_node')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_topic(self, topic_name, msg_type, timeout_sec=45.0):
        """Wait for a topic to publish at least one message."""
        received = []
        sub = self.node.create_subscription(msg_type, topic_name, received.append, 10)
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_subscription(sub)
        return received

    def test_joint_states_published(self):
        """Verify /joint_states is published."""
        from sensor_msgs.msg import JointState
        received = self._wait_for_topic('/joint_states', JointState, timeout_sec=45.0)
        self.assertTrue(received, '/joint_states not received within 45 seconds')

    def test_all_joints_present(self):
        """Verify /joint_states contains all expected joints."""
        from sensor_msgs.msg import JointState
        received = self._wait_for_topic('/joint_states', JointState, timeout_sec=45.0)
        self.assertTrue(received)

        joint_names = received[0].name
        expected_joints = [
            'arm_joint_1', 'arm_joint_2',
            'wheel_joint_1', 'wheel_joint_2',
            'gripper_joint_1', 'gripper_joint_2',
        ]
        for joint in expected_joints:
            self.assertIn(joint, joint_names,
                          f"Expected joint '{joint}' not found in {joint_names}")

    def test_all_three_controllers_active(self):
        """Verify arm_controller, wheel_controller, gripper_controller are all active."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=45.0))

        required = {'arm_controller', 'wheel_controller', 'gripper_controller'}
        deadline = time.time() + 30.0
        controller_states = {}
        while time.time() < deadline:
            future = client.call_async(ListControllers.Request())
            inner_deadline = time.time() + 5.0
            while time.time() < inner_deadline and not future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                controller_states = {c.name: c.state for c in future.result().controller}
                if required.issubset(controller_states) and all(
                    controller_states[c] == 'active' for c in required
                ):
                    break
            time.sleep(0.1)

        for controller in required:
            self.assertIn(controller, controller_states,
                          f"'{controller}' not found. Available: {list(controller_states.keys())}")
            self.assertEqual(controller_states[controller], 'active',
                             f"'{controller}' state: {controller_states[controller]}")

    def test_joint_state_broadcaster_active(self):
        """Verify joint_state_broadcaster is active."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=45.0))

        future = client.call_async(ListControllers.Request())
        deadline = time.time() + 15.0
        while time.time() < deadline and not future.done():
            rclpy.spin_once(self.node, timeout_sec=0.1)

        controller_states = {c.name: c.state for c in future.result().controller}
        self.assertEqual(
            controller_states.get('joint_state_broadcaster', 'MISSING'), 'active'
        )

    def test_emergency_stop_service_available(self):
        """Verify /emergency_stop service is available in mixed mode too."""
        from std_srvs.srv import SetBool
        client = self.node.create_client(SetBool, '/emergency_stop')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            '/emergency_stop service not available'
        )
        self.node.destroy_client(client)

    def test_dynamic_joint_states_published(self):
        """Verify /dynamic_joint_states is published (custom interfaces)."""
        from control_msgs.msg import DynamicJointState
        received = self._wait_for_topic(
            '/dynamic_joint_states', DynamicJointState, timeout_sec=45.0)
        self.assertTrue(received, '/dynamic_joint_states not received within 45 seconds')
