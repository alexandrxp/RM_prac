#!/usr/bin/env python3
#################################################################################
# Copyright 2019 ROBOTIS CO., LTD.
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
#################################################################################
#
# # Authors: Ryan Shim, Gilbert, ChanHyeong Lee

import os
import random
import math
import subprocess
import sys
import time

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.node import Node
from std_srvs.srv import Empty

from turtlebot3_msgs.srv import Goal


ROS_DISTRO = os.environ.get('ROS_DISTRO')
if ROS_DISTRO == 'humble':
    from gazebo_msgs.srv import DeleteEntity
    from gazebo_msgs.srv import SpawnEntity
    from geometry_msgs.msg import Pose


class GazeboInterface(Node):

    def __init__(self, stage_num):
        super().__init__('gazebo_interface')
        self.stage = int(stage_num)

        self.entity_name = 'goal_box'
        # fixed goal inside the garage
        self.entity_pose_x = 1.0
        self.entity_pose_y = 1.0
        # orientation for garage opening: +x
        self.dock_yaw = 0.0
        # orientation of garage opening (yaw), default -x
        self.dock_yaw = math.pi

        if ROS_DISTRO == 'humble':
            self.entity = None
            self.open_entity()
            self.delete_entity_client = self.create_client(DeleteEntity, 'delete_entity')
            self.spawn_entity_client = self.create_client(SpawnEntity, 'spawn_entity')
            self.reset_simulation_client = self.create_client(Empty, 'reset_simulation')

        self.callback_group = MutuallyExclusiveCallbackGroup()
        self.initialize_env_service = self.create_service(
            Goal,
            'initialize_env',
            self.initialize_env_callback,
            callback_group=self.callback_group
        )
        self.task_succeed_service = self.create_service(
            Goal,
            'task_succeed',
            self.task_succeed_callback,
            callback_group=self.callback_group
        )
        self.task_failed_service = self.create_service(
            Goal,
            'task_failed',
            self.task_failed_callback,
            callback_group=self.callback_group
        )

    def open_entity(self):
        try:
            package_share = get_package_share_directory('turtlebot3_gazebo')
            model_path = os.path.join(
                package_share, 'models', 'turtlebot3_dqn_world', 'goal_box', 'model.sdf'
            )
            with open(model_path, 'r') as f:
                self.entity = f.read()
            self.get_logger().info('Loaded entity from: ' + model_path)
        except Exception as e:
            self.get_logger().error('Failed to load entity file: {}'.format(e))
            raise e

    def spawn_entity(self):
        if ROS_DISTRO == 'humble':
            entity_pose = Pose()
            entity_pose.position.x = self.entity_pose_x
            entity_pose.position.y = self.entity_pose_y
            # set orientation quaternion from dock_yaw (+x)
            entity_pose.orientation.z = math.sin(self.dock_yaw / 2.0)
            entity_pose.orientation.w = math.cos(self.dock_yaw / 2.0)

            spawn_req = SpawnEntity.Request()
            spawn_req.name = self.entity_name
            spawn_req.xml = self.entity
            spawn_req.initial_pose = entity_pose

            while not self.spawn_entity_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('service for spawn_entity is not available, waiting ...')
            future = self.spawn_entity_client.call_async(spawn_req)
            rclpy.spin_until_future_complete(self, future)
            print(f'Spawn Goal at ({self.entity_pose_x}, {self.entity_pose_y}, {0.0})')
        else:
            service_name = '/world/dqn/create'
            package_share = get_package_share_directory('turtlebot3_gazebo')
            model_path = os.path.join(
                package_share, 'models', 'turtlebot3_dqn_world', 'goal_box', 'model.sdf'
            )
            # include orientation quaternion in pose
            qz = math.sin(self.dock_yaw / 2.0)
            qw = math.cos(self.dock_yaw / 2.0)
            req = (
                f'sdf_filename: "{model_path}", '
                f'name: "{self.entity_name}", '
                f'pose: {{ position: {{ '
                f'x: {self.entity_pose_x}, '
                f'y: {self.entity_pose_y}, '
                f'z: 0.0 }}, orientation: {{ x: 0.0, y: 0.0, z: {qz}, w: {qw} }} }}'
            )
            cmd = [
                'gz', 'service',
                '-s', service_name,
                '--reqtype', 'gz.msgs.EntityFactory',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', req
            ]
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                print(f'Spawn Goal at ({self.entity_pose_x}, {self.entity_pose_y}, {0.0})')
            except subprocess.CalledProcessError:
                pass

    def delete_entity(self):
        if ROS_DISTRO == 'humble':
            delete_req = DeleteEntity.Request()
            delete_req.name = self.entity_name

            while not self.delete_entity_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn('service for delete_entity is not available, waiting ...')
            future = self.delete_entity_client.call_async(delete_req)
            rclpy.spin_until_future_complete(self, future)
            print('Delete Goal')
        else:
            service_name = '/world/dqn/remove'
            req = f'name: "{self.entity_name}", type: 2'
            cmd = [
                'gz', 'service',
                '-s', service_name,
                '--reqtype', 'gz.msgs.Entity',
                '--reptype', 'gz.msgs.Boolean',
                '--timeout', '1000',
                '--req', req
            ]
            try:
                subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)
                print('Delete Goal')
            except subprocess.CalledProcessError:
                pass

    def reset_simulation(self):
        reset_req = Empty.Request()

        while not self.reset_simulation_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('service for reset_simulation is not available, waiting ...')

        self.reset_simulation_client.call_async(reset_req)

    def reset_burger(self):
        service_name_delete = '/world/dqn/remove'
        req_delete = 'name: "burger", type: 2'
        cmd_delete = [
            'gz', 'service',
            '-s', service_name_delete,
            '--reqtype', 'gz.msgs.Entity',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', req_delete
        ]
        try:
            subprocess.run(cmd_delete, check=True, stdout=subprocess.DEVNULL)
            print('Delete Burger')
        except subprocess.CalledProcessError:
            pass
        time.sleep(0.2)
        service_name_spawn = '/world/dqn/create'
        package_share = get_package_share_directory('turtlebot3_gazebo')
        model_path = os.path.join(package_share, 'models', 'turtlebot3_burger', 'model.sdf')
        req_spawn = (
            f'sdf_filename: "{model_path}", '
            f'name: "burger", '
            f'pose: {{ position: {{ x: 0.0, y: 0.0, z: 0.0 }} }}'
        )
        cmd_spawn = [
            'gz', 'service',
            '-s', service_name_spawn,
            '--reqtype', 'gz.msgs.EntityFactory',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '1000',
            '--req', req_spawn
        ]
        try:
            subprocess.run(cmd_spawn, check=True, stdout=subprocess.DEVNULL)
            print('Spawn Burger')
        except subprocess.CalledProcessError:
            pass

    def task_succeed_callback(self, request, response):
        self.delete_entity()
        time.sleep(0.2)
        self.generate_goal_pose()
        time.sleep(0.2)
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

    def task_failed_callback(self, request, response):
        self.delete_entity()
        time.sleep(0.2)
        if ROS_DISTRO == 'humble':
            self.reset_simulation()
        else:
            self.reset_burger()
        time.sleep(0.2)
        self.generate_goal_pose()
        time.sleep(0.2)
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

    def initialize_env_callback(self, request, response):
        self.delete_entity()
        time.sleep(0.2)
        if ROS_DISTRO == 'humble':
            self.reset_simulation()
        else:
            self.reset_burger()
        time.sleep(0.2)
        self.spawn_entity()
        response.pose_x = self.entity_pose_x
        response.pose_y = self.entity_pose_y
        response.success = True
        return response

    def generate_goal_pose(self):
        self.entity_pose_x = 1.0
        self.entity_pose_y = 1.0


def main(args=None):
    rclpy.init(args=sys.argv)
    stage_num = sys.argv[1] if len(sys.argv) > 1 else '1'
    gazebo_interface = GazeboInterface(stage_num)
    try:
        while rclpy.ok():
            rclpy.spin_once(gazebo_interface, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        gazebo_interface.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
