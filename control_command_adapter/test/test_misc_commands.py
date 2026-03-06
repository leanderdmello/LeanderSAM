# Copyright 2025 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import unittest
import json
import time
from websockets.sync.client import connect

import launch
import launch_ros
import launch_testing.actions

import rclpy
from interface_msgs.srv import Listing
from interface_msgs.msg import RobotStatus


def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='control_command_adapter',
                    namespace='',
                    executable='control_command_adapter',
                    name='control_command_adapter',
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=0.5, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )


wait_time = 0.05  # seconds


class TestMiscCommands(unittest.TestCase):
    callback_called = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('misc_stub')
        self.node.create_service(
            Listing, 'get_software_versions', self.get_software_versions)
        self.node.create_subscription(
            RobotStatus, 'robot_status', self.robot_status_changed, 10)

    def send_request(self, id, method):
        with connect("ws://localhost:9090") as websocket:
            x = {
                "id": id,
                "jsonrpc": "2.0",
                "method": method,
            }
            websocket.send(json.dumps(x))
            end_time = time.time() + wait_time
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=wait_time)
            return websocket.recv()

    def send_request_with_params_no_response(self, id, method, params):
        with connect("ws://localhost:9090") as websocket:
            x = {
                "id": id,
                "jsonrpc": "2.0",
                "method": method,
                "params": params
            }
            websocket.send(json.dumps(x))
            end_time = time.time() + wait_time
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=wait_time)

    def send_request_with_params(self, id, method, params):
        with connect("ws://localhost:9090") as websocket:
            x = {
                "id": id,
                "jsonrpc": "2.0",
                "method": method,
                "params": params
            }
            websocket.send(json.dumps(x))
            end_time = time.time() + wait_time
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=wait_time)
            return websocket.recv()

    def tearDown(self):
        self.node.destroy_node()
        self.callback_called = False

    def get_software_versions(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.data = ["robot: 1.4.5", "SW: 2.4.5"]
        return response

    def test_get_software_versions(self):
        """Test wether the forwarding of get sw versions works"""
        response = self.send_request(13, "GetSoftwareVersions")
        parsed_json = json.loads(response)
        result = parsed_json["result"]
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["id"], 13)
        self.assertEqual(result["SW"], "2.4.5")
        self.assertEqual(result["robot"], "1.4.5")

    def robot_status_changed(self, message):
        self.callback_called = True
        self.assertEqual(message.robot_is_moving, True)
        if message.hardware_flags != 0:
            self.assertEqual(message.hardware_flags, 7)

    def test_robot_status_changed(self):
        """Test wether the forwarding of robot status works"""
        self.send_request_with_params_no_response(13, "ReportRobotStatus", {
                                                  "RobotIsMoving": True, "OptionalHardwarePresent": ["climbing_aid", "long_flippers", "short_flippers"]})
        self.assertTrue(self.callback_called)

    def test_robot_status_changed_works_without_optional_hardware_present(self):
        """Test wether the forwarding of robot status works"""
        self.send_request_with_params_no_response(
            13, "ReportRobotStatus", {"RobotIsMoving": True})
        self.assertTrue(self.callback_called)

    def test_robot_status_changed_works_with_empty_optional_hardware_present(self):
        """Test wether the forwarding of robot status works"""
        self.send_request_with_params_no_response(13, "ReportRobotStatus", {
                                                  "RobotIsMoving": True,  "OptionalHardwarePresent": []})
        self.assertTrue(self.callback_called)

    def test_robot_status_changed_error_when_no_robot_moving(self):
        """Test wether the forwarding of robot status works"""
        response = self.send_request_with_params(
            13, "ReportRobotStatus", {"OptionalHardwarePresent": []})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 13)
