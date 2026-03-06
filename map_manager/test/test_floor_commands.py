# Copyright 2025 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import os
import unittest
import tempfile

import launch
import launch_ros
import launch_testing.actions

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from std_srvs.srv import Trigger
from interface_msgs.srv import SetString, Listing, ActionOnIndex, GetIndex, UpdateStringAt
from slam_manager_msgs.msg import SlamMode
from slam_manager_msgs.srv import SetSlamMode

temp_dir = tempfile.TemporaryDirectory()
version_file = temp_dir.name + "/versions.txt"
mode_change_topic = "/slam/set_slam_mode"

def generate_test_description():
    return (
        launch.LaunchDescription(
            [
                # Nodes under test
                launch_ros.actions.Node(
                    package='map_manager',
                    namespace='',
                    executable='map_manager',
                    name='map_manager',
                    parameters=[{
                        "map_export_path": temp_dir.name,
                        "version_file_path": version_file,
                        "map_path":  temp_dir.name,
                        "topics.subscriptions.set_slam_mode": mode_change_topic,
                    }]
                ),
                # Launch tests 0.5 s later
                launch.actions.TimerAction(
                    period=2.0, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )


wait_time = 5.0  # seconds

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
        self.floor_callback_group = MutuallyExclusiveCallbackGroup()
        self.slam_group = MutuallyExclusiveCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.node = rclpy.create_node('floor_stub')
        self.executor.add_node(self.node)
        self.add_floor_client = self.node.create_client(
            SetString, 'add_floor',callback_group=self.floor_callback_group)
        self.delete_floor_client = self.node.create_client(
            ActionOnIndex, 'delete_floor',callback_group=self.floor_callback_group)
        self.clear_floor_client = self.node.create_client(
            Trigger, 'clear_floor',callback_group=self.floor_callback_group)
        self.get_floors_client = self.node.create_client(
            Listing, 'get_floors',callback_group=self.floor_callback_group)
        self.get_mapping_floor_client = self.node.create_client(
            GetIndex, 'get_mapping_floor',callback_group=self.floor_callback_group)
        self.get_view_floor_client = self.node.create_client(
            GetIndex, 'get_view_floor',callback_group=self.floor_callback_group)
        self.select_view_floor_client = self.node.create_client(
            ActionOnIndex, 'select_view_floor',callback_group=self.floor_callback_group)
        self.select_mapping_floor_client = self.node.create_client(
            ActionOnIndex, 'select_mapping_floor',callback_group=self.floor_callback_group)
        self.set_floor_name_client = self.node.create_client(
            UpdateStringAt, 'set_floor_name',callback_group=self.floor_callback_group)
        self.clear_map_client = self.node.create_client(
            Trigger, 'clear_map',callback_group=self.floor_callback_group)
        self.get_sw_versions_client = self.node.create_client(
            Listing, 'get_software_versions',callback_group=self.floor_callback_group)
        self.slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_group)
        req = Trigger.Request()
        self.slam_mode = SlamMode.IDLE

        self.clear_map_client.wait_for_service(timeout_sec=10.0)
        req = Trigger.Request()
        self.future = self.clear_map_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)


    def tearDown(self):
        self.node.destroy_node()

    def state_changed(self, request, response):
        self.slam_mode = request.mode.mode  
        response.message = "State changed"
        response.success = True
        return response
    
    #Helper section
    def add_floor(self, name):
        req = SetString.Request()
        req.data = name
        self.future = self.add_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def delete_floor(self, index):
        req = ActionOnIndex.Request()
        req.index = index
        self.future = self.delete_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def clear_floor(self):
        req = Trigger.Request()
        self.future = self.clear_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def get_floors(self):
        req = Listing.Request()
        self.future = self.get_floors_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def get_mapping_floor(self):
        req = GetIndex.Request()
        self.future = self.get_mapping_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def get_view_floor(self):
        req = GetIndex.Request()
        self.future = self.get_view_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def select_mapping_floor(self, index):
        req = ActionOnIndex.Request()
        req.index = index
        self.future = self.select_mapping_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()
    
    def select_view_floor(self, index):
        req = ActionOnIndex.Request()
        req.index = index
        self.future = self.select_view_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()

    def set_floor_name(self, index, name):
        req = UpdateStringAt.Request()
        req.index = index
        req.data = name
        self.future = self.set_floor_name_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()

    def get_sw_versions(self):
        req = Listing.Request()
        self.future = self.get_sw_versions_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        return self.future.result()

    def find_floor_by_name(self, name):
        req = Listing.Request()
        self.future = self.get_floors_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, self.future)
        floors = self.future.result().data
        for index, floor in enumerate(floors):
            if floor == name:
                return index
        return -1

    def create_sw_version_file(self):
        with open(version_file, "w") as f:
            f.write("map_manager: 1.0\n")
            f.write("control_command_adapter: 1.0\n")
            f.write("stub_node: 0.3\n")
            f.write("visualization_node: 0.3\n")

    def wait_state_changed(self, expected_state):
        start = self.node.get_clock().now()    
        while (self.slam_mode != expected_state) and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (self.node.get_clock().now() - start).nanoseconds > wait_time * 1e9:
                break  
        self.assertEqual(self.slam_mode, expected_state)

    #Test section
    def test_add_delete_floor(self):
        """
        GIVEN a floor is added
        WHEN we delete the floor
        THEN the api should return an empty list
        """
        #Given a floor is added
        self.add_floor('floor 1')
        self.wait_state_changed(SlamMode.MAPPING)
        floors = self.get_floors()
        self.assertEqual(floors.data, ['floor 1'])

        #When we delete the floor
        self.delete_floor(0)
        self.wait_state_changed(SlamMode.IDLE)

        #Then the api should report an empty list
        floors = self.get_floors()
        self.assertEqual(floors.data, [])
        
    def test_add_floor_already_axists_error(self):
        """
        GIVEN a floor is added
        WHEN we add another floor with the same name
        THEN the api should report an error
        """
        #Given a floor is added 
        self.add_floor('floor 1')

        #When we add another floor with the same name
        response = self.add_floor('floor 1')

        #Then the api should report an error
        self.assertFalse(response.success)
    
    def test_delete_floor_gives_index_out_of_range_error(self):
        """
        WHEN deleting a none existing floor
        THEN the api should report an error
        """
        #When deleting a none existing floor
        response = self.delete_floor(10)

        #Then the api should report an error
        self.assertFalse(response.success)
        
    def test_add_rename_floor(self):
        """
        WHEN a floor is added
        AND then we rename this floor
        THEN the api should report the new name
        """
        #When a floor is added
        self.add_floor('floor 1')
        self.wait_state_changed(SlamMode.MAPPING)
        floors = self.get_floors()
        self.assertEqual(floors.data, ['floor 1'])

        #And then we rename this floor
        self.set_floor_name(0, 'different floor name')

        #Then the api should report the new name
        floors = self.get_floors()
        self.assertEqual(floors.data, ['different floor name'])

    def test_rename_floor_gives_index_out_of_range_error(self):
        """
        WHEN we rename a floor that does not exist
        THEN the api should report an error
        """
        #When we rename a floor that does not exist
        response = self.set_floor_name(10, "new name")

        #Then the api should report an error
        self.assertFalse(response.success)
 
    def test_rename_floor_already_exists(self):
        """
        GIVEN 2 floors are added
        WHEN we rename floor 2 to the same name as floor 1
        THEN the api should report an error
        """
        #Given 2 floors are added
        floor1 = 'floor 1'
        floor2 = 'floor 2'
        self.add_floor(floor1)
        self.add_floor(floor2)

        #When we rename floor 2 to the same name as floor 1
        index = self.find_floor_by_name(floor1)
        response = self.set_floor_name(index, floor2)

        #Then the api should report an error
        self.assertFalse(response.success)

    def test_select_mapping_floor(self):
        """
        GIVEN a floor is added
        WHEN a mapping floor is selected
        THEN the state should go to localizing
        AND get mapping floor should return the selected floor
        """
        #Given a floor is added
        self.add_floor('floor 1')
        self.wait_state_changed(SlamMode.MAPPING)

        #When a mapping floor is selected
        self.select_mapping_floor(0)

        #Then the state should go to localizing
        self.wait_state_changed(SlamMode.LOCALIZATION)

        #And get mapping floor should return the selected floor
        result = self.get_mapping_floor()
        self.assertEqual(result.index, 0)

    def test_select_mapping_floor_index_out_of_bounds(self):
        """
        WHEN a none existing mapping floor is selected
        THEN the api should report an error
        """
        response = self.select_mapping_floor(0)
        self.assertFalse(response.success)

    def test_get_mapping_floor_not_set(self):
        """
        WHEN requesting the mapping floor with no floors created
        THEN the api should report an error
        """
        #When requesting the mapping floor with no floors created
        response = self.get_mapping_floor()
        
        #Then the api should report an error
        self.assertEqual(response.index, GetIndex.Response.NONE)

    def test_select_view_floor(self):
        """
        GIVEN a floor exists
        WHEN requesting the view floor
        THEN the api should return the selected floor
        """
        #Given a floor exists
        self.add_floor('floor 1')
        self.wait_state_changed(SlamMode.MAPPING)

        #When requesting the view floor
        self.select_view_floor(0)

        #Then the api should return the selected floor
        result = self.get_view_floor()
        self.assertEqual(result.index, 0)
        
    def test_select_view_floor_index_out_of_bounds(self):
        """
        WHEN selecting a none existing view floor
        THEN the api should report an error
        """
        #When selecting a none existing view floor
        response = self.select_view_floor(0)

        #Then the api should report an error
        self.assertFalse(response.success)
        
    def test_get_view_floor_not_set(self):
        """
        WHEN requesting the view floor index with no view floor selected
        THEN the api should report none
        """
        #When requesting the view floor index with no view floor selected
        response = self.get_view_floor()

        #Then the api should report none
        self.assertEqual(response.index, GetIndex.Response.NONE)
        
    def test_get_sw_versions(self):
        """
        WHEN requesting the software versions
        THEN the api should report them
        """
        #When requesting the software versions
        self.create_sw_version_file()

        #Then the api should report them
        result = self.get_sw_versions()
        self.assertEqual(result.data, ["map_manager: 1.0", "control_command_adapter: 1.0", "stub_node: 0.3", "visualization_node: 0.3"])
        
    def test_get_sw_versions_no_file(self):
        """
        WHEN requesting the software versions
        AND the file does not exist
        THEN the api should report an empty list
        """
        #When requesting the software versions and the file does not exist
        os.remove(version_file)
        result = self.get_sw_versions()

        #Then the api should report an empty list
        self.assertEqual(result.data, [])
        