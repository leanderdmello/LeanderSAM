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


class TestMalformedCommands(unittest.TestCase):
    callback_called = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('malformed_stub')

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

    def test_invalid_command(self):
        """Checks wether an invalid command return the correct error response"""
        response = self.send_request(
            12, "InvalidCommand")
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["error"]["code"], -32601)
        self.assertEqual(parsed_json["id"], 12)

    def test_invalid_json(self):
        """Checks wether an invalid command return the correct error response"""
        with connect("ws://localhost:9090") as websocket:
            websocket.send("THis is invalid json }{;][`]}")
            end_time = time.time() + wait_time
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=wait_time)
            parsed_json = json.loads(websocket.recv())
        self.assertEqual(parsed_json["error"]["code"], 1)

    def test_no_command(self):
        with connect("ws://localhost:9090") as websocket:
            x = {
                "id": 12,
                "jsonrpc": "2.0"
            }
            websocket.send(json.dumps(x))
            end_time = time.time() + wait_time
            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=wait_time)
            parsed_json = json.loads(websocket.recv())
        self.assertEqual(parsed_json["error"]["code"], 1)
