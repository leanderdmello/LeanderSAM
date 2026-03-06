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
from interface_msgs.srv import ActionOnOptionalIndex
from interface_msgs.srv import Listing
from interface_msgs.srv import UpdateStringAt


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


class TestMarkerCommands(unittest.TestCase):
    callback_called = False

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('marker_stub')
        self.node.create_service(
            SetString, 'add_marker', self.add_marker)
        self.node.create_service(
            ActionOnOptionalIndex, 'remove_marker', self.remove_marker)
        self.node.create_service(
            Listing, 'get_markers', self.get_markers)
        self.node.create_service(
            SetString, 'set_marker_label', self.set_marker_label)
        self.node.create_service(
            UpdateStringAt, 'set_marker_label_at', self.set_marker_label_at)

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

    def add_marker(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.data, "marker 1")
        response.success = True
        return response

    def test_add_marker(self):
        """Checks wether add marker gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "AddMarker", {"MarkerName": "marker 1"})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def test_add_marker_error_when_marker_name_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "AddMarker", {})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)

    def remove_marker(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        if request.index > 0:
            self.assertEqual(request.index, 1)
        else:
            self.assertEqual(request.index, -1)
        response.success = True
        return response

    def test_remove_marker(self):
        """Checks wether remove marker gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "RemoveMarker", {"MarkerIndex": 1})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def test_remove_marker_without_index(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "RemoveMarker", {})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def get_markers(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.data = ["marker 1", "marker 2"]
        return response

    def test_get_markers(self):
        """Checks wether get markers gets forwarded correctly"""
        response = self.send_request(
            12, "GetMarkers")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], ["marker 1", "marker 2"])
        self.assertEqual(parsed_json["id"], 12)

    def set_marker_label(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.data, "marker 1")
        response.success = True
        return response

    def test_set_marker_label(self):
        """Checks wether set markers labels forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetMarkerLabel", {"MarkerLabel": "marker 1"})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def test_set_mark_label_error_when_marker_label_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "SetMarkerLabel", {})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)

    def set_marker_label_at(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.data, "marker 1")
        self.assertEqual(request.index, 1)
        response.success = True
        return response

    def test_set_marker_label_at(self):
        """Checks wether set markers labels forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetMarkerLabel", {"MarkerIndex": 1, "MarkerLabel": "marker 1"})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)
