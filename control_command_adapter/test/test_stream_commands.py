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
from std_srvs.srv import SetBool
from interface_msgs.srv import SetVideoResolutionMode
from interface_msgs.srv import SetVideoStreamAddress
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


class TestStreamCommands(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('stream_stub')
        self.node.create_service(
            SetBool, 'scale_unit_meters', self.scale_unit_meters)
        self.node.create_service(
            SetVideoResolutionMode, 'set_video_resolution_mode', self.set_video_resolution_mode)
        self.node.create_service(
            SetVideoStreamAddress, 'set_video_stream_address', self.set_video_stream_address)
        self.node.create_service(
            SetBool, 'set_video_stream', self.set_video_stream)
        self.node.create_service(
            Trigger, 'clear_path', self.clear_path)

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

    def scale_unit_meters(self, request, response):
        print(request)
        print(response)
        self.assertTrue(request.data)
        response.success = True
        return response

    def test_scale_unit_meters(self):
        """Checks wether set scale unit gets forwarded correctly"""
        self.send_request_with_params(
            12, "SetScaleUnit", {"ScaleUnit": "meters"})

    def set_video_resolution_mode(self, request, response):
        print(request)
        print(response)
        self.assertEqual(request.mode, 1)
        response.success = True
        return response

    def test_set_video_resolution_mode(self):
        """Checks wether set set video resolution mode gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetVideoResolutionMode", {"ResolutionMode": "highres"})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def set_video_stream_address(self, request, response):
        print(request)
        print(response)
        self.assertEqual(request.ip_address, "127.0.0.1")
        self.assertEqual(request.port, 9090)
        response.success = True
        return response

    def test_set_video_stream_address(self):
        """Checks wether set set video stream address gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetVideoStreamAddress", {"IpAddress": "127.0.0.1", "Port": 9090})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def set_video_stream(self, request, response):
        print(request)
        print(response)
        self.assertTrue(request.data)
        response.success = True
        return response

    def test_set_video_stream_option(self):
        """Checks wether set set video stream option gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetVideoStreamOption", {"VideoStreamOption": True})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def clear_path(self, request, response):
        print(request)
        print(response)
        response.success = True
        return response

    def test_clear_path(self):
        """Checks wether set set video stream option gets forwarded correctly"""
        response = self.send_request(
            12, "ClearPath")
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)
