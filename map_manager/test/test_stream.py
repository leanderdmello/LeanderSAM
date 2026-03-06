# Copyright 2025 Avular Holding B.V.
# All Rights Reserved
# You may use this code under the terms of the Avular
# Software End-User License Agreement.
#
# You should have received a copy of the Avular
# Software End-User License Agreement license with
# this file, or download it from: avular.com/eula

import os
import shutil
import unittest
import tempfile
import time
from websockets.sync.client import connect

import launch
import launch_ros
import launch_testing.actions
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from interface_msgs.srv import SetString, Listing, ActionOnIndex, GetIndex, UpdateStringAt
from interface_msgs.msg import PointCloudAvailable, Trajectories
from slam_manager_msgs.msg import SlamMode
from slam_manager_msgs.srv import SetSlamMode

from std_msgs.msg import Header
from std_srvs.srv import Trigger
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

temp_dir = tempfile.TemporaryDirectory()
version_file = temp_dir.name + "/versions.txt"

map_topic = "/slam/mapping/path"
path_topic = "/slam/mapping/map_global"
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
                        "topics.subscriptions.global_map" : map_topic,
                        "topics.subscriptions.trajectory" : path_topic,
                        "services.client.set_slam_mode" : mode_change_topic,
                        "slam_timeout" : 60000
                    }]
                ),
                launch.actions.TimerAction(
                    period=5.0, actions=[launch_testing.actions.ReadyToTest()]),
            ]
        ), {},
    )


wait_time = 5.0  # seconds


class TestMapStreaming(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.callback_called = False
        self.floor_callback_group = MutuallyExclusiveCallbackGroup()
        self.slam_group = MutuallyExclusiveCallbackGroup()
        self.slam_mode_group = MutuallyExclusiveCallbackGroup()
        self.executor = MultiThreadedExecutor()
        self.node = rclpy.create_node('floor_slam_stub')
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

        self.path_message = None
        self.pointcloud_message = None
        self.pointcloud_callback_called = False
        self.path_callback_called = False
        self.clear_map_client.wait_for_service(timeout_sec=10.0)
        self.clear_map()

    def clear_map(self):
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)
        req = Trigger.Request()
        future = self.clear_map_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        self.node.destroy_service(slam_state_service)

    #Helper section
    def tearDown(self):
        self.node.destroy_node()

    def add_floor(self, name):
        req = SetString.Request()
        req.data = name
        future = self.add_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()
    
    def select_view_floor(self, index):
        req = ActionOnIndex.Request()
        req.index = index
        future = self.select_view_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()
    
    def select_mapping_floor(self, index):
        req = ActionOnIndex.Request()
        req.index = index
        future = self.select_mapping_floor_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result()
    
    def create_pointcloud(self, name="map"):
        fields = []
        px = PointField()
        px.name = 'x'
        px.offset = 0
        px.datatype = PointField.FLOAT32
        px.count = 1
        fields.append(px)

        py = PointField()
        py.name = 'y'
        py.offset = 4
        py.datatype = PointField.FLOAT32
        py.count = 1
        fields.append(py)

        pz = PointField()
        pz.name = 'z'
        pz.offset = 8
        pz.datatype = PointField.FLOAT32
        pz.count = 1
        fields.append(pz)

        pi = PointField()
        pi.name = 'intensity'
        pi.offset = 12
        pi.datatype = PointField.FLOAT32
        pi.count = 1
        fields.append(pi)

        points = []
        x = float(1)
        y = float(1)
        z = float(1)
        i = float(1)
        point = [x, y, z, i]
        points.append(point)

        header = Header()
        header.frame_id = name
        header.stamp = rclpy.time.Time().to_msg()
        return point_cloud2.create_cloud(header, fields, points)

    def create_path(self, value=1.0):
        path = Path()
        header = Header()
        header.frame_id = "path"
        header.stamp = rclpy.time.Time().to_msg()
        path.header = header

        pose_stamped = PoseStamped()
        pose_stamped.header = header
        pose_stamped.pose.position.x = value
        pose_stamped.pose.position.y = value
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0

        path.poses.append(pose_stamped)
        return path
    
    def create_default_map(self, pointcloud_publisher):
        self.pointcloud_callback_called = False
        pointcloud_publisher.publish(self.create_pointcloud("disk map"))
        self.wait_pointcloud_callback()

    def create_default_path(self, path_publisher):
        self.path_callback_called = False
        path_publisher.publish(self.create_path(2.0))    
        self.wait_path_callback()
 
    def state_changed(self, request, response):
        self.slam_mode = request.mode.mode  
        response.message = "State changed"
        response.success = True
        return response

    def wait_state_changed(self, expected_state):
        end_time = time.time() + wait_time
        while (self.slam_mode != expected_state) and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (time.time() > end_time):
                break  
        self.assertEqual(self.slam_mode, expected_state)

    def wait_path_callback(self):
        end_time = time.time() + wait_time
        while (not self.path_callback_called) and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (time.time() > end_time):
                break  

    def wait_pointcloud_callback(self):
        end_time = time.time() + wait_time
        while (not self.pointcloud_callback_called) and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if (time.time() > end_time):
                break  

    def path_received(self, message):
        self.path_callback_called = True
        self.path_message = message

    def pointcloud_received(self, message):
        self.pointcloud_callback_called = True
        self.pointcloud_message = message

    def find_floor_by_name(self, name):
        req = Listing.Request()
        future = self.get_floors_client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        floors = future.result().data
        for index, floor in enumerate(floors):
            if floor == name:
                return index
        return -1
  
    def get_path(self, folder):
        return temp_dir.name + f"/map_1/{folder}"
  
    #Test section
    def test_mapping_streams_pointcloud_for_view_floor(self):
        """
        GIVEN multiple floors are available
        WHEN we are in mapping
        AND a view floor is selected and a pointcloud is send
        THEN the pointcloud should be forwarded to the map visualizer
        """
        pointcloud_publisher = self.node.create_publisher(PointCloud2, map_topic, 1)
        pointcloud_subscriber = self.node.create_subscription(PointCloudAvailable, "point_cloud_available", lambda message: self.pointcloud_received(message), 1, callback_group=self.slam_group)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #Given multiple floors are added
        self.add_floor(name="Floor 1")
        self.add_floor(name="Floor 2")
        self.wait_state_changed(SlamMode.MAPPING)

        #When a view floor is selected and a pointcloud is send
        #Needed because list order is not guaranteed
        index = self.find_floor_by_name("Floor 2")
        self.pointcloud_callback_called = False
        self.select_view_floor(index)
        pointcloud_publisher.publish(self.create_pointcloud())

        #Then the pointcloud should be forwarded to the map visualizer
        self.wait_pointcloud_callback()
        self.assertTrue(self.pointcloud_callback_called)
        self.assertEqual(self.pointcloud_message.floor_path, self.get_path('Floor 2'))

        self.node.destroy_publisher(pointcloud_publisher)
        self.node.destroy_subscription(pointcloud_subscriber)
        self.node.destroy_service(slam_state_service)
        
    def test_mapping_streams_path_for_view_floor(self):
        """
        GIVEN multiple floors are available
        WHEN we are in mapping
        AND a view floor is selected and a path is send
        THEN the path should be forwarded to the map visualizer
        """
        path_publisher = self.node.create_publisher(Path, path_topic, 1)
        path_subscriber = self.node.create_subscription(Trajectories,"trajectories", lambda message: self.path_received(message), 1, callback_group=self.slam_group)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #Given multiple floors are added
        self.add_floor(name="Floor 1")
        self.add_floor(name="Floor 2")
        self.wait_state_changed(SlamMode.MAPPING)

        #When a view floor is selected and a pointcloud is send
        index = self.find_floor_by_name("Floor 2")
        self.select_view_floor(index)

        self.path_callback_called = False
        path_publisher.publish(self.create_path())

        #Then the path should be forwarded to the map visualizer
        self.wait_path_callback()
        self.assertTrue(self.path_callback_called)    
        self.assertEqual(len(self.path_message.trajectories), 1)

        self.node.destroy_publisher(path_publisher)
        self.node.destroy_subscription(path_subscriber)
        self.node.destroy_service(slam_state_service)

    def test_localization_ignores_map_data(self):
        """
        GIVEN a floor with a map and path is available
        WHEN we are in localizing
        AND a pointcloud is received from SLAM
        THEN this pointcloud is ignored
        """
        pointcloud_publisher = self.node.create_publisher(PointCloud2, map_topic, 1)
        pointcloud_subscriber = self.node.create_subscription(PointCloudAvailable, "point_cloud_available", lambda message: self.pointcloud_received(message), 1, callback_group=self.slam_group)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #Given a floor with a map and path is available
        self.add_floor(name="Floor 1")
        index = self.find_floor_by_name("Floor 1")
        self.select_view_floor(index)
        self.create_default_map(pointcloud_publisher)
        self.wait_state_changed(SlamMode.MAPPING)

        #When we are in localizing
        index = self.find_floor_by_name("Floor 1")
        self.select_mapping_floor(index)
        self.wait_state_changed(SlamMode.LOCALIZATION)

        #And a pointcloud is received from SLAM
        self.pointcloud_callback_called = False
        pointcloud_publisher.publish(self.create_pointcloud())

        #Then this pointcloud is ignored
        self.wait_pointcloud_callback()
        self.assertFalse(self.pointcloud_callback_called)  

        self.node.destroy_publisher(pointcloud_publisher)
        self.node.destroy_subscription(pointcloud_subscriber)
        self.node.destroy_service(slam_state_service)

    def test_localization_accepts_path_data(self):
        """
        GIVEN a floor with a map and path is available
        WHEN we are in localizing
        AND a path is received from SLAM
        THEN this is forwarded to the map visualizer
        """
        path_publisher = self.node.create_publisher(Path, path_topic, 1)
        path_subscriber = self.node.create_subscription(Trajectories,"trajectories", lambda message: self.path_received(message), 1, callback_group=self.slam_group)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #Given a floor with a map and path is available
        self.expected_floor_path = self.get_path('Floor 1')
        self.add_floor(name="Floor 1")
        index = self.find_floor_by_name("Floor 1")
        self.select_view_floor(index)
        self.create_default_path(path_publisher)
        self.wait_state_changed(SlamMode.MAPPING)

        #When we are in localizing
        index = self.find_floor_by_name("Floor 1")
        self.select_mapping_floor(index)
        self.wait_state_changed(SlamMode.LOCALIZATION)

        #And a path is received from SLAM
        self.path_callback_called = False
        path_publisher.publish(self.create_path())

        #Then this is forwarded to the map visualizer
        self.wait_path_callback()
        self.assertTrue(self.path_callback_called)  
        self.assertEqual(len(self.path_message.trajectories), 2)

        self.node.destroy_publisher(path_publisher)
        self.node.destroy_subscription(path_subscriber)
        self.node.destroy_service(slam_state_service)

    def test_idle_ignores_map_data(self):
        """
        WHEN we are in Idle
        AND a pointcloud is received from SLAM
        THEN this pointcloud is ignored
        """
        pointcloud_publisher = self.node.create_publisher(PointCloud2, map_topic, 1)
        pointcloud_subscriber = self.node.create_subscription(PointCloudAvailable, "point_cloud_available", lambda message: self.pointcloud_received(message), 1, callback_group=self.slam_group)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #When we are in Idle and a pointcloud is received from SLAM
        self.pointcloud_callback_called = False
        pointcloud_publisher.publish(self.create_pointcloud())

        #Then this pointcloud is ignored
        self.wait_pointcloud_callback()
        self.assertFalse(self.pointcloud_callback_called) 

        self.node.destroy_publisher(pointcloud_publisher)
        self.node.destroy_subscription(pointcloud_subscriber)
        self.node.destroy_service(slam_state_service)
        
    def test_idle_ignores_path_data(self):
        """
        WHEN we are in Idle
        AND a path is received from SLAM
        THEN this path is ignored
        """
        path_publisher = self.node.create_publisher(Path, path_topic, 1)
        path_subscriber = self.node.create_subscription(Trajectories,"trajectories", lambda message: self.path_received(message), 1, callback_group=self.slam_group)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #When we are in Idle and a path is received from SLAM
        self.path_callback_called = False
        path_publisher.publish(self.create_path())

        #Then this path is ignored
        self.wait_path_callback()   
        self.assertFalse(self.path_callback_called)

        self.node.destroy_publisher(path_publisher)
        self.node.destroy_subscription(path_subscriber)
        self.node.destroy_service(slam_state_service)

    def test_multiple_trajectories(self):
        """
        GIVEN a floor with a map and path is available
        WHEN we are in Localizing
        AND a path is received from SLAM
        THEN this path together with the path from disk is send to the map visualizer
        """
        path_publisher = self.node.create_publisher(Path, path_topic, 1)
        path_subscriber = self.node.create_subscription(Trajectories,"trajectories", self.path_received, 1)
        slam_state_service = self.node.create_service(SetSlamMode, mode_change_topic, self.state_changed, callback_group=self.slam_mode_group)

        #Given a floor with a map and path is available
        self.add_floor(name="Floor 1")
        index = self.find_floor_by_name("Floor 1")
        self.select_view_floor(index)
        self.create_default_path(path_publisher)
        self.wait_state_changed(SlamMode.MAPPING)

        #When we are in Localizing
        self.select_mapping_floor(index)
        self.wait_state_changed(SlamMode.LOCALIZATION)

        #And a path is received from SLAM
        self.path_callback_called = False
        self.expected_path_size = 2
        path_publisher.publish(self.create_path(3.4))

        #Then this path together with the path from disk is send to the map visualizer
        self.wait_path_callback()   
        self.assertTrue(self.path_callback_called)
        self.assertEqual(len(self.path_message.trajectories), 2)

        self.node.destroy_publisher(path_publisher)
        self.node.destroy_subscription(path_subscriber)
        self.node.destroy_service(slam_state_service)