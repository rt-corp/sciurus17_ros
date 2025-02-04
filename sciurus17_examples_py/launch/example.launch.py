# Copyright 2024 RT Corporation
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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from sciurus17_description.robot_description_loader import RobotDescriptionLoader


def generate_launch_description():

    description_loader = RobotDescriptionLoader()
    declare_loaded_description = DeclareLaunchArgument(
        'loaded_description',
        default_value=description_loader.load(),
        description='Set robot_description text.  \
                    It is recommended to use RobotDescriptionLoader() \
                    in sciurus17_description.')

    moveit_config = (MoveItConfigsBuilder('sciurus17').planning_scene_monitor(
        publish_robot_description=True,
        publish_robot_description_semantic=True,
        ).moveit_cpp(file_path=get_package_share_directory('sciurus17_examples_py') +
                     '/config/sciurus17_moveit_py_examples.yaml').to_moveit_configs())

    moveit_config.robot_description = {
        'robot_description': LaunchConfiguration('loaded_description')
    }

    moveit_config.move_group_capabilities = {'capabilities': ''}

    declare_example_name = DeclareLaunchArgument(
        'example',
        default_value='gripper_control',
        description=('Set an example executable name: '
                     '[gripper_control, neck_control, waist_control, '
                     'pick_and_place_right_arm_waist, pick_and_place_left_arm]'))

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description=('Set true when using the gazebo simulator.')
    )

    # 下記Issue対応のためここでパラメータを設定する
    # https://github.com/moveit/moveit2/issues/2940#issuecomment-2401302214
    config_dict = moveit_config.to_dict()
    config_dict.update({'use_sim_time': LaunchConfiguration('use_sim_time')})

    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='sciurus17_examples_py',
        executable=LaunchConfiguration('example'),
        output='screen',
        parameters=[config_dict],
    )

    return LaunchDescription([
        declare_loaded_description,
        declare_use_sim_time,
        declare_example_name,
        example_node
    ])
