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
from interface_msgs.srv import Listing
from interface_msgs.srv import ActionOnIndex
from std_srvs.srv import Trigger
from interface_msgs.srv import GetIndex
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


class TestFloorCommands(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.callback_called = False
        self.view_floor_index = -1
        self.mapping_floor_index = -1
        self.node = rclpy.create_node('floor_stub')
        self.node.create_service(
            SetString, 'add_floor', self.add_floor)
        self.node.create_service(
            ActionOnIndex, 'delete_floor', self.delete_floor)
        self.node.create_service(
            Trigger, 'clear_floor', self.clear_floor)
        self.node.create_service(
            Listing, 'get_floors', self.get_floors)
        self.node.create_service(
            GetIndex, 'get_mapping_floor', self.get_mapping_floor)
        self.node.create_service(
            GetIndex, 'get_view_floor', self.get_view_floor)
        self.node.create_service(
            ActionOnIndex, 'select_view_floor', self.select_view_floor)
        self.node.create_service(
            ActionOnIndex, 'select_mapping_floor', self.select_mapping_floor)
        self.node.create_service(
            UpdateStringAt, 'set_floor_name', self.set_floor_name)

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

    def add_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.data, "floor 1")
        response.success = True
        return response

    def test_add_floor(self):
        """Checks wether add floor gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "AddFloor", {"FloorName": "floor 1"})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def test_add_floor_error_when_floor_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "AddFloor", {})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)

    def get_floors(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.data = ["floor1", "floor2"]
        return response

    def test_get_floors(self):
        """Test wether the forwarding of get floors works"""
        response = self.send_request(13, "GetFloors")
        parsed_json = json.loads(response)
        result = parsed_json["result"]
        self.assertTrue(self.callback_called)
        self.assertTrue("floor1" in result)
        self.assertTrue("floor2" in result)
        self.assertEqual(len(result), 2)
        self.assertEqual(parsed_json["id"], 13)

    def delete_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.index, 2)
        response.success = True
        return response

    def test_delete_floor(self):
        """Checks wether delete floor gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "DeleteFloor", {"FloorIndex": 2})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def test_delete_floor_error_when_floor_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "DeleteFloor", {})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)

    def clear_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.success = True
        return response

    def test_clear_floor(self):
        """Checks wether clear floor gets forwarded correctly"""
        response = self.send_request(
            12, "ClearFloor")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], None)
        self.assertEqual(parsed_json["id"], 12)

    def get_mapping_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.index = self.mapping_floor_index
        return response

    def test_get_mapping_floor(self):
        """Checks wether get mapping floor gets forwarded correctly"""
        self.mapping_floor_index = 5
        response = self.send_request(
            12, "GetMappingFloor")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], 5)
        self.assertEqual(parsed_json["id"], 12)
        
    def test_get_mapping_floor_not_set(self):
        """Checks wether get mapping floor gets forwarded correctly"""
        self.mapping_floor_index = -1
        response = self.send_request(
            12, "GetMappingFloor")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 2)
        self.assertEqual(parsed_json["id"], 12)

    def get_view_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        response.index = self.view_floor_index
        return response

    def test_get_view_floor(self):
        """Checks wether get view floor gets forwarded correctly"""
        self.view_floor_index = 5
        response = self.send_request(
            12, "GetViewFloor")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["result"], 5)
        self.assertEqual(parsed_json["id"], 12)
        
    def test_get_view_floor_not_set(self):
        """Checks wether get view floor gets forwarded correctly"""
        self.view_floor_index = -1
        response = self.send_request(
            12, "GetViewFloor")
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 2)
        self.assertEqual(parsed_json["id"], 12)

    def select_view_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.index, 2)
        return response

    def test_select_view_floor(self):
        """Checks wether select view floor gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SelectViewFloor", {"FloorIndex": 2})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["id"], 12)
        
    def test_select_view_floor_error_when_floor_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "SelectViewFloor", {})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)

    def select_mapping_floor(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.index, 2)
        return response

    def test_select_mapping_floor(self):
        """Checks wether select mapping floor gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SelectMappingFloor", {"FloorIndex": 2})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["id"], 12)

    def test_select_mapping_floor_error_when_floor_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "SelectMappingFloor", {})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)

    def set_floor_name(self, request, response):
        print(request)
        print(response)
        self.callback_called = True
        self.assertEqual(request.index, 2)
        self.assertEqual(request.data, "floor 1")
        return response

    def test_set_floor_name(self):
        """Checks wether set floor name gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetFloorName", {"FloorIndex": 2, "FloorName": "floor 1"})
        parsed_json = json.loads(response)
        self.assertTrue(self.callback_called)
        self.assertEqual(parsed_json["id"], 12)

    def test_set_floor_name_error_when_params_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request(
            12, "SetFloorName")
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], -32602)
        self.assertEqual(parsed_json["id"], 12)

    def test_set_floor_name_error_when_floor_name_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "SetFloorName", {"FloorIndex": 2})
        parsed_json = json.loads(response)
        self.assertFalse(self.callback_called)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)
