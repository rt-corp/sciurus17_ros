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
import time
import math

# generic ros libraries
import rclpy
from rclpy.logging import get_logger
from geometry_msgs.msg import Pose, Point, Quaternion

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

def plan_and_execute(
    robot,
    planning_component,
    logger,
    single_plan_parameters=None,
    multi_plan_parameters=None,
    sleep_time=0.0,
):
    """Helper function to plan and execute a motion."""
    # plan to goal
    logger.info("Planning trajectory")
    if multi_plan_parameters is not None:
        plan_result = planning_component.plan(
            multi_plan_parameters=multi_plan_parameters
        )
    elif single_plan_parameters is not None:
        plan_result = planning_component.plan(
            single_plan_parameters=single_plan_parameters
        )
    else:
        plan_result = planning_component.plan()

    # execute the plan
    if plan_result:
        logger.info("Executing plan")
        robot_trajectory = plan_result.trajectory
        robot.execute(robot_trajectory, controllers=[])
    else:
        logger.error("Planning failed")

    time.sleep(sleep_time)

def main(args=None):
    rclpy.init(args=args)
    logger = get_logger("moveit_py.pick_and_place_left_arm")

    # instantiate MoveItPy instance and get planning component
    sciurus17 = MoveItPy(node_name="pick_and_place_left_arm")
    logger.info("MoveItPy instance created")

    # 左腕制御用 planning component
    l_arm = sciurus17.get_planning_component("l_arm_group")
    # 左グリッパ制御用 planning component
    l_gripper = sciurus17.get_planning_component("l_gripper_group")

    robot_model = sciurus17.get_robot_model()
    
    plan_request_params = PlanRequestParameters(
        sciurus17, 
        "ompl_rrtc",
    )
    gripper_plan_request_params = PlanRequestParameters(
        sciurus17, 
        "ompl_rrtc",
    )
    # 動作速度の調整
    plan_request_params.max_acceleration_scaling_factor = 0.1 # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 0.1 # Set 0.0 ~ 1.0
    # 動作速度の調整
    gripper_plan_request_params.max_acceleration_scaling_factor = 1.0 # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor = 1.0 # Set 0.0 ~ 1.0

    # グリッパの開閉角
    GRIPPER_CLOSE = math.radians(0.0)
    GRIPPER_OPEN = math.radians(-40.0)
    GRIPPER_GRASP = math.radians(-20.0)
    # 物体を掴む位置
    PICK_POSE = Pose(position=Point(x=0.25, y=0.0, z=0.12))
    # 物体を置く位置
    PLACE_POSE = Pose(position=Point(x=0.35, y=0.0, z=0.12))
    # 物体を持ち上げる高さ
    LIFTING_HEIFHT = 0.25
    
    # SRDFに定義されている"l_arm_init_pose"の姿勢にする
    l_arm.set_start_state_to_current_state()
    l_arm.set_goal_state(configuration_name="l_arm_init_pose")
    # plan to goal
    plan_and_execute(
        sciurus17,
        l_arm, logger,
        single_plan_parameters=plan_request_params,
        sleep_time=3.0,
    )

    # 何かを掴んでいた時のためにハンドを開く
    l_gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions("l_gripper_group", [GRIPPER_OPEN])
    l_gripper.set_goal_state(robot_state=robot_state)
    # plan to goal
    plan_and_execute(
        sciurus17,
        l_gripper, logger,
        single_plan_parameters=gripper_plan_request_params,
        sleep_time=3.0,
    )
    # 物体の上に腕を伸ばす
    # 掴みに行く
    # ハンドを閉じる
    # 持ち上げる
    # 移動する
    # 下ろす
    # ハンドを開く
    # ハンドを持ち上げる
    # SRDFに定義されている"l_arm_init_pose"の姿勢にする
    # ハンドを閉じる

    
    # Finishe with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()