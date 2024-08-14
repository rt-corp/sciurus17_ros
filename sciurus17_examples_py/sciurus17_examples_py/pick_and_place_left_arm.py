import time

# generic ros libraries
import rclpy
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
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
    l_arm = sciurus17.get_planning_component("l_arm_group")
    logger.info("MoveItPy instance created")
    
    # set plan start state using predefined state
    l_arm.set_start_state_to_current_state()

    # set pose goal using predefined state
    l_arm.set_goal_state(configuration_name="l_arm_init_pose")

    # plan to goal
    plan_and_execute(sciurus17, l_arm, logger, sleep_time=3.0)
    
    # related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()