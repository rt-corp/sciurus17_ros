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
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch
from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch
from moveit_configs_utils.launches import generate_rsp_launch
from sciurus17_description.robot_description_loader import RobotDescriptionLoader
from ament_index_python.packages import get_package_share_directory


from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ld = LaunchDescription()
    description_loader = RobotDescriptionLoader()

    ld.add_action(
        DeclareLaunchArgument(
            'loaded_description',
            default_value=description_loader.load(),
            description='Set robot_description text.  \
                        It is recommended to use RobotDescriptionLoader() in sciurus17_description.'
        )
    )

    moveit_config = (
        MoveItConfigsBuilder("sciurus17")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .moveit_cpp(
            file_path=get_package_share_directory("sciurus17_examples_py")
            + "/config/sciurus17_moveit_py_examples.yaml"
        )
        .to_moveit_configs()
        )

    moveit_config.robot_description = {
        'robot_description': LaunchConfiguration('loaded_description')
        }

    moveit_config.move_group_capabilities = {
        "capabilities": ""
        }

    declare_example_name = DeclareLaunchArgument(
        'example', default_value='gripper_control',
        description=('Set an example executable name: '
                     '[gripper_control, neck_control, waist_control,'
                     'pick_and_place_right_arm_waist, pick_and_place_left_arm]')
    )

    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
                        package='sciurus17_examples_py',
                        executable=LaunchConfiguration('example'),
                        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    ld.add_action(declare_example_name)
    ld.add_action(example_node)

    return ld
