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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    #    This format doesn't work because we have to expand gzpose into
    #    different args for spawn_entity.py
    #    gz_pose = DeclareLaunchArgument(
    #        'gzpose', default_value='-x 0 -y 0 -z 0.0 -R 0.0 -P 0.0 -Y 0.0 ',
    #        description='Spawn gazebo position as provided to spawn_entity.py'
    #    )

    # @TODO: load PID gains? used in gazebo_ros_control fork
    # @TODO: load tiago_pal_hardware_gazebo
    ld = LaunchDescription()

    declare_launch_arguments(ld)
    declare_actions(ld)

    return ld


def declare_launch_arguments(launch_description: LaunchDescription):
    model_name = DeclareLaunchArgument(
        'robot_name', default_value='tiago',
        description='Gazebo model name'
    )
    launch_description.add_action(model_name)
    return


def declare_actions(launch_description: LaunchDescription):

    robot_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', LaunchConfiguration('robot_name'),
                                   # LaunchConfiguration('gzpose'),
                                   ],
                        output='screen')
    launch_description.add_action(robot_entity)

    return
