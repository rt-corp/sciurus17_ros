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
import math


# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
# generic ros libraries
import rclpy
from rclpy.logging import get_logger

from sciurus17_examples_py.utils import plan_and_execute


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('moveit_py.neck_control')

    # instantiate MoveItPy instance and get planning component
    sciurus17 = MoveItPy(node_name='neck_control')
    logger.info('MoveItPy instance created')

    # 首制御用 planning component
    neck = sciurus17.get_planning_component('neck_group')
    planning_scene_monitor = sciurus17.get_planning_scene_monitor()

    robot_model = sciurus17.get_robot_model()

    plan_request_params = PlanRequestParameters(
        sciurus17,
        'ompl_rrtc_default',
    )
    # 動作速度の調整
    plan_request_params.max_acceleration_scaling_factor = 0.1    # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 0.1    # Set 0.0 ~ 1.0

    # SRDFに定義されている'neck_init_pose'の姿勢にする
    neck.set_start_state_to_current_state()
    neck.set_goal_state(configuration_name='neck_init_pose')
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 現在角度をベースに、目標角度を作成する
    joint_values = []
    with planning_scene_monitor.read_only() as scene:
        robot_state = scene.current_state
        joint_values = robot_state.get_joint_group_positions('neck_group')

    joint_values[0] = math.radians(45.0)
    neck.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('neck_group', joint_values)
    neck.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    joint_values[0] = math.radians(-45.0)
    neck.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('neck_group', joint_values)
    neck.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    joint_values[0] = math.radians(0.0)
    neck.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('neck_group', joint_values)
    neck.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    joint_values[1] = math.radians(45.0)
    neck.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('neck_group', joint_values)
    neck.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    joint_values[1] = math.radians(-45.0)
    neck.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('neck_group', joint_values)
    neck.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    neck.set_start_state_to_current_state()
    neck.set_goal_state(configuration_name='neck_init_pose')
    plan_and_execute(
        sciurus17,
        neck,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # Finishe with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
