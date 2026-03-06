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
from interface_msgs.srv import SetString
from std_srvs.srv import Trigger


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


class TestMapCommands(unittest.TestCase):
    callback_called = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('map_stub')
        self.node.create_service(
            SetString, 'export_map', self.export_map)
        self.node.create_service(
            Trigger, 'clear_map', self.clear_map)

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

    def tearDown(self):
        self.node.destroy_node()
        self.callback_called = False

    def export_map(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.success = True
        return response

    def test_export_map(self):
        """Checks wether export map gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "ExportMap", {"Label": "label"})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def test_export_map_works_gives_error_when_no_label(self):
        """Checks wether export map gets forwarded correctly"""
        response = self.send_request(
            12, "ExportMap")
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], -32602)
        self.assertEqual(parsed_json["id"], 12)

    def clear_map(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.success = True
        return response

    def test_clear_map(self):
        """Checks wether clear map gets forwarded correctly"""
        response = self.send_request(
            12, "ClearMap")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["id"], 12)
