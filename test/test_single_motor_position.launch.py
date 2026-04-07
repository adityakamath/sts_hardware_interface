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
Launch test for single_motor_position example in mock mode.

Verifies that the complete ros2_control stack starts successfully
with a single position-mode motor in mock mode (no hardware required).
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
    """Generate launch description for the single motor position integration test."""
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory

    pkg_share = get_package_share_directory('sts_hardware_interface')
    single_motor_position_launch = os.path.join(
        pkg_share, 'launch', 'single_motor_position.launch.py')

    single_motor_position = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(single_motor_position_launch),
        launch_arguments={
            'use_mock': 'true',
            'gui': 'false',
        }.items()
    )

    return launch.LaunchDescription([
        single_motor_position,
        launch_testing.actions.ReadyToTest(),
    ])


class TestSingleMotorPositionLaunch(unittest.TestCase):
    """Test that single motor position stack launches and basic topics are available."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_single_motor_position_node')

    def tearDown(self):
        self.node.destroy_node()

    def _wait_for_topic(self, topic_name, msg_type, timeout_sec=30.0):
        """Wait for a topic to publish at least one message."""
        received = []

        def callback(msg):
            received.append(msg)

        sub = self.node.create_subscription(msg_type, topic_name, callback, 10)
        deadline = time.time() + timeout_sec

        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        return len(received) > 0

    def test_joint_states_published(self):
        """Verify /joint_states is published after stack starts."""
        from sensor_msgs.msg import JointState
        self.assertTrue(
            self._wait_for_topic('/joint_states', JointState, timeout_sec=30.0),
            '/joint_states not received within 30 seconds'
        )

    def test_joint_state_has_arm_joint(self):
        """Verify /joint_states contains arm_joint."""
        from sensor_msgs.msg import JointState
        received = []

        sub = self.node.create_subscription(
            JointState, '/joint_states', received.append, 10)

        deadline = time.time() + 30.0
        while time.time() < deadline and not received:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(sub)
        self.assertTrue(received, '/joint_states not received')
        self.assertIn('arm_joint', received[0].name)

    def test_controller_manager_available(self):
        """Verify /controller_manager/list_controllers service is reachable."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            '/controller_manager/list_controllers service not available'
        )
        self.node.destroy_client(client)

    def test_arm_controller_active(self):
        """Verify arm_controller is in 'active' state."""
        from controller_manager_msgs.srv import ListControllers
        client = self.node.create_client(ListControllers, '/controller_manager/list_controllers')
        self.assertTrue(client.wait_for_service(timeout_sec=30.0))

        # Poll until arm_controller reaches 'active' state or timeout.
        deadline = time.time() + 30.0
        controller_states = {}
        while time.time() < deadline:
            future = client.call_async(ListControllers.Request())
            poll_deadline = time.time() + 5.0
            while time.time() < poll_deadline and not future.done():
                rclpy.spin_once(self.node, timeout_sec=0.1)
            if future.done():
                controller_states = {c.name: c.state for c in future.result().controller}
                if controller_states.get('arm_controller') == 'active':
                    break
            time.sleep(0.5)

        self.node.destroy_client(client)
        self.assertIn('arm_controller', controller_states,
                      f'arm_controller not found. '
                      f'Available: {list(controller_states.keys())}')
        self.assertEqual(controller_states['arm_controller'], 'active',
                         f'arm_controller state: {controller_states["arm_controller"]}')

    def test_emergency_stop_service_available(self):
        """Verify /emergency_stop service is reachable."""
        from std_srvs.srv import SetBool
        client = self.node.create_client(SetBool, '/emergency_stop')
        self.assertTrue(
            client.wait_for_service(timeout_sec=30.0),
            '/emergency_stop service not available'
        )
        self.node.destroy_client(client)
