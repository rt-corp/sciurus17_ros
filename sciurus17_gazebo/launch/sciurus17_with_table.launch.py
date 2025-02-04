# Copyright 2023 RT Corporation
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from sciurus17_description.robot_description_loader import RobotDescriptionLoader


def generate_launch_description():
    declare_use_head_camera = DeclareLaunchArgument(
        'use_head_camera',
        default_value='true',
        description='Use head camera.'
    )

    declare_use_chest_camera = DeclareLaunchArgument(
        'use_chest_camera',
        default_value='true',
        description='Use chest camera.'
    )

    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'GZ_SIM_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'GZ_SIM_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('sciurus17_description'))}
    world_file = os.path.join(
        get_package_share_directory('sciurus17_gazebo'), 'worlds', 'table.sdf')
    gui_config = os.path.join(
        get_package_share_directory('sciurus17_gazebo'), 'gui', 'gui.config')
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    gz_sim = ExecuteProcess(
            cmd=['gz sim -r', world_file, '--gui-config', gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    gz_sim_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'sciurus17',
                   '-z', '1.02',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_gazebo = 'true'
    description_loader.use_gazebo_head_camera = LaunchConfiguration('use_head_camera')
    description_loader.use_gazebo_chest_camera = LaunchConfiguration('use_chest_camera')
    description_loader.gz_control_config_package = 'sciurus17_control'
    description_loader.gz_control_config_file_path = 'config/sciurus17_controllers.yaml'
    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('sciurus17_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    spawn_joint_state_broadcaster = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_right_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner right_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_right_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner right_gripper_controller'],
                shell=True,
                output='screen',
            )

    spawn_left_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner left_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_left_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner left_gripper_controller'],
                shell=True,
                output='screen',
            )

    spawn_neck_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner neck_controller'],
                shell=True,
                output='screen',
            )

    spawn_waist_yaw_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner waist_yaw_controller'],
                shell=True,
                output='screen',
            )

    bridge_file = os.path.join(
        get_package_share_directory('sciurus17_gazebo'), 'config', 'bridge.yaml')

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                parameters=[{'config_file': bridge_file}],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        declare_use_head_camera,
        declare_use_chest_camera,
        gz_sim,
        gz_sim_spawn_entity,
        spawn_joint_state_broadcaster,
        spawn_right_arm_controller,
        spawn_right_gripper_controller,
        spawn_left_arm_controller,
        spawn_left_gripper_controller,
        spawn_neck_controller,
        spawn_waist_yaw_controller,
        move_group,
        bridge
    ])
