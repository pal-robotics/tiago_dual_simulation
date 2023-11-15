# Copyright (c) 2022 PAL Robotics S.L. All rights reserved.
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

import os
from os import environ, pathsep
from typing import Dict
from ament_index_python.packages import get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable

from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_pal.include_utils import include_scoped_launch_py_description

from launch_ros.actions import Node


def generate_launch_description():

    # @TODO: review pal_gazebo
    # @TODO: review tiago_spawn
    # @TODO: simulation_tiago_bringup?
    # @TODO: pal_pcl_points_throttle_and_filter

    # Create the launch description and populate
    ld = LaunchDescription()

    launch_args = declare_launch_arguments()

    for arg in launch_args.values():
        ld.add_action(arg)

    declare_actions(ld, launch_args)

    return ld


def declare_launch_arguments() -> Dict:

    arg_dict = {}

    robot_name_arg = DeclareLaunchArgument(
        name='robot_name',
        default_value='tiago_dual',
        description='Robot name'
    )
    arg_dict[robot_name_arg.name] = robot_name_arg

    arm_right = DeclareLaunchArgument(
        'arm_type_right',
        default_value='tiago-arm',
        description='Which type of the right arm.',
        choices=['no-arm', 'tiago-arm', 'sea'])

    arg_dict[arm_right.name] = arm_right

    arm_left = DeclareLaunchArgument(
        'arm_type_left',
        default_value='tiago-arm',
        description='Which type of the left arm.',
        choices=['no-arm', 'tiago-arm', 'sea'])

    arg_dict[arm_left.name] = arm_left

    end_effector_right = DeclareLaunchArgument(
        'end_effector_right',
        default_value='pal-gripper',
        description='End effector model of the right arm.',
        choices=['pal-gripper', 'pal-hey5', 'custom', 'no-end-effector'])

    arg_dict[end_effector_right.name] = end_effector_right

    end_effector_left = DeclareLaunchArgument(
        'end_effector_left',
        default_value='pal-gripper',
        description='End effector model of the left arm.',
        choices=['pal-gripper', 'pal-hey5', 'custom', 'no-end-effector'])

    arg_dict[end_effector_left.name] = end_effector_left

    ft_sensor_right = DeclareLaunchArgument(
        'ft_sensor_right',
        default_value='schunk-ft',
        description='FT sensor model. ',
        choices=['schunk-ft', 'no-ft-sensor'])

    arg_dict[ft_sensor_right.name] = ft_sensor_right

    ft_sensor_left = DeclareLaunchArgument(
        'ft_sensor_left',
        default_value='schunk-ft',
        description='FT sensor model. ',
        choices=['schunk-ft', 'no-ft-sensor'])

    arg_dict[ft_sensor_left.name] = ft_sensor_left

    wrist_model_right = DeclareLaunchArgument(
        'wrist_model_right',
        default_value='wrist-2010',
        description='Wrist model. ',
        choices=['wrist-2010', 'wrist-2017'])

    arg_dict[wrist_model_right.name] = wrist_model_right

    wrist_model_left = DeclareLaunchArgument(
        'wrist_model_left',
        default_value='wrist-2010',
        description='Wrist model. ',
        choices=['wrist-2010', 'wrist-2017'])

    arg_dict[wrist_model_left.name] = wrist_model_left

    camera_model = DeclareLaunchArgument(
        'camera_model',
        default_value='orbbec-astra',
        description='Head camera model. ',
        choices=['no-camera', 'orbbec-astra', 'orbbec-astra-pro', 'asus-xtion'])

    arg_dict[camera_model.name] = camera_model

    laser_model = DeclareLaunchArgument(
        'laser_model',
        default_value='sick-571',
        description='Base laser model. ',
        choices=['no-laser', 'sick-571', 'sick-561', 'sick-551', 'hokuyo'])

    arg_dict[laser_model.name] = laser_model

    has_screen = DeclareLaunchArgument(
        'has_screen',
        default_value='false',
        description='Define if the robot has a screen. ',
        choices=['true', 'false'])

    arg_dict[has_screen.name] = has_screen

    base_type = DeclareLaunchArgument(
        'base_type',
        default_value='pmb2',
        description='Define base type of the robot. ',
        choices=['pmb2', 'omni_base'])

    arg_dict[base_type.name] = base_type

    navigation_arg = DeclareLaunchArgument(
        name='navigation',
        default_value='false',
        description='Specify if launching Navigation2'
    )
    arg_dict[navigation_arg.name] = navigation_arg

    moveit_arg = DeclareLaunchArgument(
        name='moveit',
        default_value='false',
        description='Specify if launching MoveIt2'
    )
    arg_dict[moveit_arg.name] = moveit_arg

    world_name_arg = DeclareLaunchArgument(
        name='world_name',
        default_value='pal_office',
        description="Specify world name, we'll convert to full path"
    )
    arg_dict[world_name_arg.name] = world_name_arg

    use_sim_time = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='True',
        description="Use sim time"
    )
    arg_dict[use_sim_time.name] = use_sim_time

    return arg_dict


def declare_actions(launch_description: LaunchDescription, launch_args: Dict):

    packages = ['tiago_dual_description', 'tiago_description',
                'pmb2_description', 'hey5_description', 'pal_gripper_description']

    model_path = get_model_paths(packages)

    gazebo_model_path_env_var = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH', model_path)

    gazebo = include_scoped_launch_py_description(
        pkg_name='pal_gazebo_worlds',
        paths=['launch', 'pal_gazebo.launch.py'],
        launch_args=[launch_args['world_name'], gazebo_model_path_env_var],
        launch_configurations={
            "world_name":  LaunchConfiguration("world_name"),
            "model_paths": packages,
            "resource_paths": packages,
        })

    launch_description.add_action(gazebo)

    navigation = include_scoped_launch_py_description(
        pkg_name='tiago_dual_2dnav',
        paths=['launch', 'tiago_dual_sim_nav_bringup.launch.py'],
        launch_configurations={
            "robot_name":  LaunchConfiguration("robot_name"),
            "is_public_sim":  LaunchConfiguration("public_sim"),
            "laser":  LaunchConfiguration("laser_model")},
        condition=IfCondition(LaunchConfiguration('navigation')))

    launch_description.add_action(navigation)

    move_group = include_scoped_launch_py_description(
        pkg_name='tiago_dual_moveit_config',
        paths=['launch', 'move_dual_group.launch.py'],
        launch_configurations={
            "robot_name": LaunchConfiguration("robot_name"),
            "use_sim_time": LaunchConfiguration("use_sim_time")},
        condition=IfCondition(LaunchConfiguration('moveit')))

    launch_description.add_action(move_group)

    robot_spawn = include_scoped_launch_py_description(
        pkg_name='tiago_dual_gazebo',
        paths=['launch', 'robot_spawn.launch.py'],
        launch_configurations={'robot_name': LaunchConfiguration('robot_name'), })

    launch_description.add_action(robot_spawn)

    tiago_bringup = include_scoped_launch_py_description(
        pkg_name='tiago_dual_bringup', paths=['launch', 'tiago_dual_bringup.launch.py'],
        launch_configurations={
            'use_sim_time': LaunchConfiguration("use_sim_time"),
            "robot_name": LaunchConfiguration("robot_name"),
            "arm_type_right": LaunchConfiguration("arm_type_right"),
            "arm_type_left": LaunchConfiguration("arm_type_left"),
            "end_effector_right": LaunchConfiguration("end_effector_right"),
            "end_effector_left": LaunchConfiguration("end_effector_left"),
            "ft_sensor_right": LaunchConfiguration("ft_sensor_right"),
            "ft_sensor_left": LaunchConfiguration("ft_sensor_left"),
            "wrist_model_right": LaunchConfiguration("wrist_model_right"),
            "wrist_model_left": LaunchConfiguration("wrist_model_left"),
            "laser_model": LaunchConfiguration("laser_model"),
            "camera_model": LaunchConfiguration("camera_model"),
            "base_type": LaunchConfiguration("base_type"),
            "has_screen": LaunchConfiguration("has_screen")}
    )

    launch_description.add_action(tiago_bringup)

    tuck_arm = Node(package='tiago_gazebo',
                    executable='tuck_arm.py',
                    emulate_tty=True,
                    output='both')

    launch_description.add_action(tuck_arm)

    return


def get_model_paths(packages_names):
    model_paths = ''
    for package_name in packages_names:
        if model_paths != '':
            model_paths += pathsep

        package_path = get_package_prefix(package_name)
        model_path = os.path.join(package_path, 'share')

        model_paths += model_path

    if 'GAZEBO_MODEL_PATH' in environ:
        model_paths += pathsep + environ['GAZEBO_MODEL_PATH']

    return model_paths


def get_resource_paths(packages_names):
    resource_paths = ''
    for package_name in packages_names:
        if resource_paths != '':
            resource_paths += pathsep

        package_path = get_package_prefix(package_name)
        resource_paths += package_path

    if 'GAZEBO_RESOURCE_PATH' in environ:
        resource_paths += pathsep + environ['GAZEBO_RESOURCE_PATH']

    return resource_paths
