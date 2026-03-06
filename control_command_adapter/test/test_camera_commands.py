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
from interface_msgs.srv import CameraMovement
from interface_msgs.srv import FollowRobot
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


class TestCameraCommands(unittest.TestCase):
    callback_called = None

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('camera_stub')
        self.node.create_service(
            CameraMovement, 'modify_camera', self.modify_camera)
        self.node.create_service(
            Trigger, 'move_camera_to_robot', self.move_camera_to_robot)
        self.node.create_service(
            FollowRobot, 'follow_robot', self.follow_robot)
        self.node.create_service(
            Trigger, 'set_robot_location_to_camera', self.set_robot_location_to_camera)

    def tearDown(self):
        self.node.destroy_node()
        self.callback_called = None

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

    def modify_camera(self, request, response):
        print(request)
        print(response)
        self.callback_called = "modify_camera"
        if request.update_zoom:
            self.assertEqual(request.dzoom, 0.123)
        if request.update_x:
            self.assertEqual(request.dx, 1.9)
        if request.update_y:
            self.assertEqual(request.dy, -2.3)
        if request.update_pan:
            self.assertEqual(request.dpan, 0.127)
        if request.update_tilt:
            self.assertEqual(request.dtilt, 25.3)

        response.success = True
        return response

    def test_modify_camera(self):
        """Checks wether modify camera gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "ModifyCamera", {"Zoom": 0.123, "XTranslation": 1.9, "YTranslation": -2.3, "Pan": 0.127, "Tilt": 25.3})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "modify_camera")

    def test_modify_camera_is_accepted_when_params_omitted(self):
        """Modify camera also works with no params"""
        response = self.send_request(
            12, "ModifyCamera")
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "modify_camera")

    def test_modify_camera_is_accepted_when_params_empty_omitted(self):
        """Modify camera also works with empty params"""
        response = self.send_request_with_params(
            12, "ModifyCamera", {})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "modify_camera")

    def move_camera_to_robot(self, request, response):
        response.success = True
        self.callback_called = "move_camera_to_robot"
        return response

    def test_move_camera_to_robot(self):
        """Checks wether move camera to robot gets forwarded correctly"""
        response = self.send_request(
            12, "MoveCameraToRobotLocation")
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "move_camera_to_robot")

    def follow_robot(self, request, response):
        self.assertTrue(request.follow_type >= 0 and request.follow_type <= 2)
        self.callback_called = "follow_robot"
        response.success = True
        return response

    def test_follow_robot_off(self):
        """Checks wether follow robot gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetFollowRobotOption", {"FollowRobot": "off"})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "follow_robot")

    def test_follow_robot_position(self):
        """Checks wether follow robot gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetFollowRobotOption", {"FollowRobot": "position"})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "follow_robot")

    def test_follow_robot_position_and_orientation(self):
        """Checks wether follow robot gets forwarded correctly"""
        response = self.send_request_with_params(
            12, "SetFollowRobotOption", {"FollowRobot": "position_and_orientation"})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "follow_robot")

    def test_follow_robot_error_when_floor_omitted(self):
        """Check that when a parameter is missing an error is thrown"""
        response = self.send_request_with_params(
            12, "SetFollowRobotOption", {})
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["error"]["code"], 1)
        self.assertEqual(parsed_json["id"], 12)
        self.assertIsNone(self.callback_called)

    def set_robot_location_to_camera(self, request, response):
        response.success = True
        self.callback_called = "set_robot_location_to_camera"
        return response

    def test_set_robot_location_to_camera(self):
        """Checks wether set robot location to camera gets forwarded correctly"""
        response = self.send_request(
            12, "SetRobotLocationToCameraPosition")
        parsed_json = json.loads(response)
        self.assertEqual(parsed_json["id"], 12)
        self.assertEqual(self.callback_called, "set_robot_location_to_camera")
