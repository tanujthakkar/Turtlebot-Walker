"""
 * MIT License

Copyright (c) 2022 Tanuj Thakkar
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

"""

import os

from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    record = LaunchConfiguration('record', default='false')

    rosbag_record = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen',
        condition=IfCondition(record)
        )

    pkg_turtlebot3_gazebo = FindPackageShare(
        package='turtlebot3_gazebo').find('turtlebot3_gazebo')

    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_turtlebot3_gazebo,
                'launch',
                'turtlebot3_world.launch.py')
        ))

    walker = Node(
        package='turtlebot_walker',
        executable='turtlebot_walker',
        name='walker_node',
        output='screen',
        )

    ld = LaunchDescription()
    ld.add_action(rosbag_record)
    ld.add_action(turtlebot3_gazebo)
    ld.add_action(walker)

    return ld
