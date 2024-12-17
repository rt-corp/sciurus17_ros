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
import copy
import math

from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
import rclpy
from rclpy.logging import get_logger
from sciurus17_examples_py.utils import plan_and_execute
from std_msgs.msg import Header


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('moveit_py.pick_and_place_left_arm')

    # instantiate MoveItPy instance and get planning component
    sciurus17 = MoveItPy(node_name='pick_and_place_left_arm')
    logger.info('MoveItPy instance created')

    # 左腕制御用 planning component
    l_arm = sciurus17.get_planning_component('l_arm_group')
    # 左グリッパ制御用 planning component
    l_gripper = sciurus17.get_planning_component('l_gripper_group')

    robot_model = sciurus17.get_robot_model()

    plan_request_params = PlanRequestParameters(
        sciurus17,
        'ompl_rrtc_default',
    )
    gripper_plan_request_params = PlanRequestParameters(
        sciurus17,
        'ompl_rrtc_default',
    )
    # 動作速度の調整
    plan_request_params.max_acceleration_scaling_factor = 0.1    # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 0.1    # Set 0.0 ~ 1.0

    # グリッパの開閉角
    GRIPPER_CLOSE = math.radians(0.0)
    GRIPPER_OPEN = math.radians(-40.0)
    GRIPPER_GRASP = math.radians(-20.0)
    # 物体を持ち上げる高さ
    LIFTING_HEIFHT = 0.25
    # 物体を掴む位置
    GRASP_POSE = Pose(position=Point(x=0.25, y=0.0, z=0.12),
                      orientation=Quaternion(x=-0.707, y=0.0, z=0.0, w=0.707))    # downward
    PRE_AND_POST_GRASP_POSE = copy.deepcopy(GRASP_POSE)
    PRE_AND_POST_GRASP_POSE.position.z += LIFTING_HEIFHT
    # 物体を置く位置
    RELEASE_POSE = Pose(position=Point(x=0.35, y=0.0, z=0.12),
                        orientation=Quaternion(x=-0.707, y=0.0, z=0.0, w=0.707))    # downward
    PRE_AND_POST_RELEASE_POSE = copy.deepcopy(RELEASE_POSE)
    PRE_AND_POST_RELEASE_POSE.position.z += LIFTING_HEIFHT

    # SRDFに定義されている'l_arm_init_pose'の姿勢にする
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(configuration_name='l_arm_init_pose')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 何かを掴んでいた時のためにハンドを開く
    l_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('l_gripper_group', [GRIPPER_OPEN])
    l_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        l_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 物体の上に腕を伸ばす
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(pose_stamped_msg=PoseStamped(header=Header(frame_id='base_link'),
                                                      pose=PRE_AND_POST_GRASP_POSE),
                         pose_link='l_link7')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 掴みに行く
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(pose_stamped_msg=PoseStamped(header=Header(frame_id='base_link'),
                                                      pose=GRASP_POSE),
                         pose_link='l_link7')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # ハンドを閉じる
    l_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('l_gripper_group', [GRIPPER_GRASP])
    l_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        l_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # 持ち上げる
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(pose_stamped_msg=PoseStamped(header=Header(frame_id='base_link'),
                                                      pose=PRE_AND_POST_GRASP_POSE),
                         pose_link='l_link7')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 移動する
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(pose_stamped_msg=PoseStamped(header=Header(frame_id='base_link'),
                                                      pose=PRE_AND_POST_RELEASE_POSE),
                         pose_link='l_link7')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # 下ろす
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(pose_stamped_msg=PoseStamped(header=Header(frame_id='base_link'),
                                                      pose=RELEASE_POSE),
                         pose_link='l_link7')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # ハンドを開く
    l_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('l_gripper_group', [GRIPPER_OPEN])
    l_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        l_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # ハンドを持ち上げる
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(pose_stamped_msg=PoseStamped(header=Header(frame_id='base_link'),
                                                      pose=PRE_AND_POST_RELEASE_POSE),
                         pose_link='l_link7')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # SRDFに定義されている'l_arm_init_pose'の姿勢にする
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(configuration_name='l_arm_init_pose')
    plan_and_execute(
        sciurus17,
        l_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # ハンドを閉じる
    l_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('l_gripper_group', [GRIPPER_CLOSE])
    l_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        sciurus17,
        l_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()
