#! /usr/bin/env python3

from functools import cache

from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle

from control_msgs.action import FollowJointTrajectory

import hello_helpers.hello_misc as hm

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from stretch_mujoco_driver.stretch_mujoco_driver import StretchMujocoDriver

from stretch_mujoco.enums.actuators import Actuators


class JointTrajectoryAction:

    def __init__(self, node: "StretchMujocoDriver", action_server_rate_hz: int):
        self.node = node
        self._goal_handle = None
        self._action_server = ActionServer(
            self.node,
            FollowJointTrajectory,
            "/stretch_controller/follow_joint_trajectory",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            callback_group=node.main_group,
        )
        self.timeout = 0.2  # seconds

        # How long to wait for an actuator to settle before aborting the goal.
        # This only applies to the MuJoCo sim driver (this class);
        # 45s keeps a genuinely stuck/obstructed base from hanging
        # forever while giving legitimate moves enough room to finish.
        self._wait_timeout = 45.0  # seconds

        self.last_goal_time = self.node.get_clock().now().to_msg()

        self.latest_goal_id = 0

    def handle_accepted_callback(self, goal_handle: ServerGoalHandle):
        # This server only allows one goal at a time
        if self._goal_handle is not None and self._goal_handle.is_active:
            self.node.get_logger().info("Aborting previous goal")
            # Abort the existing goal
            # self._goal_handle.abort() \TODO(@hello-atharva): This is causing state transition issues.
        self._goal_handle = goal_handle

        # Increment goal ID
        self.latest_goal_id += 1

        # Launch an asynch coroutine to execute the goal
        goal_handle.execute()

    def goal_callback(self, goal_request):
        self.node.get_logger().info(f"Received goal request, {goal_request}")
        new_goal_time = self.node.get_clock().now().to_msg()
        time_duration = (new_goal_time.sec + new_goal_time.nanosec * pow(10, -9)) - (
            self.last_goal_time.sec + self.last_goal_time.nanosec * pow(10, -9)
        )

        if (
            self._goal_handle is not None
            and self._goal_handle.is_active
            and (time_duration < self.timeout)
        ):
            return GoalResponse.REJECT  # Reject goal if another goal is currently active

        self.last_goal_time = self.node.get_clock().now().to_msg()
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.node.get_logger().info("Received cancel request")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.node.get_logger().info("Executing trajectory...")

        trajectory = goal_handle.request.trajectory
        # Normalize trajectory joint naming to match the real Stretch driver.
        try:
            trajectory = hm.merge_arm_joints(trajectory)
            trajectory = hm.preprocess_gripper_trajectory(trajectory)
        except hm.FollowJointTrajectoryException as e:
            self.node.get_logger().error(str(e))
            goal_handle.abort()
            result = FollowJointTrajectory.Result()
            result.error_code = e.CODE
            result.error_string = str(e)
            return result

        joint_names = trajectory.joint_names

        for point in trajectory.points:
            positions: list[float] = point.positions
            velocities: list[float | None] = (
                point.velocities if point.velocities else [None] * len(joint_names)
            )

            actuators_in_use = []

            for i, joint in enumerate(joint_names):
                try:
                    actuator = get_actuator_by_joint_names_in_command_groups(joint)
                except:
                    self.node.get_logger().error(f"No command group for joint '{joint}'")
                    continue

                target_position = positions[i]
                velocity = velocities[i]

                actuator_name = actuator.name

                if (
                    actuator_name == Actuators.left_wheel_vel.name
                    or actuator_name == Actuators.right_wheel_vel.name
                ) and velocity is not None:
                    self.node.sim.set_base_velocity(velocity, 0)
                    continue

                is_incremental_base_joint = joint in (
                    "translate_mobile_base",
                    "rotate_mobile_base",
                    "position",
                ) or actuator_name in (
                    Actuators.base_translate.name,
                    Actuators.base_rotate.name,
                )

                if is_incremental_base_joint:
                    # These command-group joints are incremental in Stretch core,
                    # so they must be executed as relative base motions in sim.
                    self.node.sim.move_by(actuator_name, target_position)
                    actuators_in_use.append(actuator_name)
                    continue

                self.node.sim.move_to(actuator_name, target_position)

                actuators_in_use.append(actuator_name)

            for actuator in actuators_in_use:
                if actuator in (Actuators.base_translate.name, Actuators.base_rotate.name):
                    reached = self.node.sim.wait_while_is_moving(actuator, timeout=self._wait_timeout)
                else:
                    reached = self.node.sim.wait_until_at_setpoint(actuator, timeout=self._wait_timeout)

                if not reached:
                    result = FollowJointTrajectory.Result()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = (
                        f"Joint '{actuator}' did not reach its commanded target "
                        "before timing out (it may be obstructed)."
                    )
                    self.node.get_logger().error(result.error_string)
                    goal_handle.abort()
                    return result

            # Simulate wait until point.time_from_start
            # self._wait_until(
            #     point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            # )

        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        self.node.get_logger().info("Trajectory execution complete")
        return result

    def _wait_until(self, seconds):
        loop_rate = self.node.create_rate(10)
        t_start = self.node.get_clock().now().seconds_nanoseconds()[0]
        while (self.node.get_clock().now().seconds_nanoseconds()[0] - t_start) < seconds:
            loop_rate.sleep()


@cache
def get_actuator_by_joint_names_in_command_groups(joint_name: str) -> Actuators:
    """
    Joint names defined by stretch_core command groups, return their Actuator here.
    """
    if joint_name == "joint_left_wheel":
        return Actuators.left_wheel_vel
    if joint_name == "joint_right_wheel":
        return Actuators.right_wheel_vel
    if joint_name == "translate_mobile_base" or joint_name == "position":
        return Actuators.base_translate
    if joint_name == "rotate_mobile_base":
        return Actuators.base_rotate

    if joint_name == "joint_lift":
        return Actuators.lift
    if joint_name == "joint_arm" or joint_name == "wrist_extension":
        return Actuators.arm
    if joint_name in ["joint_arm_l0", "joint_arm_l1", "joint_arm_l2", "joint_arm_l3"]:
        return Actuators.arm
    if joint_name == "joint_wrist_yaw":
        return Actuators.wrist_yaw
    if joint_name == "joint_wrist_pitch":
        return Actuators.wrist_pitch
    if joint_name == "joint_wrist_roll":
        return Actuators.wrist_roll
    if (
        joint_name == "joint_gripper_slide"
        or joint_name == "joint_gripper_finger_left"
        or joint_name == "joint_gripper_finger_right"
        or joint_name == "gripper_aperture"
        or joint_name == "stretch_gripper"
    ):
        return Actuators.gripper
    if joint_name == "joint_head_pan":
        return Actuators.head_pan
    if joint_name == "joint_head_tilt":
        return Actuators.head_tilt

    raise NotImplementedError(f"Actuator for {joint_name} is not defined.")
