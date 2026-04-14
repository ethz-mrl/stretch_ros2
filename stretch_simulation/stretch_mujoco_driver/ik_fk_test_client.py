#! /usr/bin/env python3

from __future__ import annotations

import argparse
from typing import Sequence

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from control_msgs.action import FollowJointTrajectory
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


ACTION_NAME = "/stretch_controller/follow_joint_trajectory"
HOME_SERVICE = "/home_the_robot"
ARM_EXTENSION_M = 0.35
GRIPPER_APERTURE_M = 0.08
TF_TIMEOUT_S = 10.0


class IKFKTestClient(Node):
    def __init__(self, point_duration_s: float):
        super().__init__("ik_fk_test_client")
        self._action_client = ActionClient(self, FollowJointTrajectory, ACTION_NAME)
        self._home_client = self.create_client(Trigger, HOME_SERVICE)
        self._point_duration = max(point_duration_s, 0.1)
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def run(self) -> int:
        self.get_logger().info(f"Waiting for action server on {ACTION_NAME}...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Action server not available.")
            return 1

        fk_names = [
            "joint_lift",
            "joint_arm",
            "joint_wrist_yaw",
            "joint_head_pan",
            "joint_head_tilt",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "gripper_aperture",
        ]
        fk_positions = [
            0.80,
            ARM_EXTENSION_M,
            1.0,
            0.60,
            -0.60,
            -0.90,
            0.60,
            GRIPPER_APERTURE_M,
        ]

        # IK/planner style for Stretch arm: 4 telescoping joints instead of wrist_extension.
        ik_names = [
            "joint_lift",
            "joint_arm_l3",
            "joint_arm_l2",
            "joint_arm_l1",
            "joint_arm_l0",
            "joint_wrist_yaw",
            "joint_head_pan",
            "joint_head_tilt",
            "joint_wrist_pitch",
            "joint_wrist_roll",
            "gripper_aperture",
        ]
        ik_positions = [
            0.80,
            ARM_EXTENSION_M / 4.0,
            ARM_EXTENSION_M / 4.0,
            ARM_EXTENSION_M / 4.0,
            ARM_EXTENSION_M / 4.0,
            1.0,
            0.60,
            -0.60,
            -0.90,
            0.60,
            GRIPPER_APERTURE_M,
        ]

        ok_fk = self._send_single_point_goal("FK", fk_names, fk_positions)
        self._log_requested_transforms("after FK goal")
        ok_ik = self._send_single_point_goal("IK", ik_names, ik_positions)
        self._log_requested_transforms("after IK goal")

        if ok_fk and ok_ik:
            self.get_logger().info("Both FK and IK tests passed.")
            return 0

        self.get_logger().error("One or more tests failed.")
        return 2

    def return_robot_home(self) -> None:
        self.get_logger().info(f"Requesting home via {HOME_SERVICE}...")
        if not self._home_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/home_the_robot service not available. Skipping home request.")
            return

        req = Trigger.Request()
        future = self._home_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=60.0)

        response = future.result()
        if response is None:
            self.get_logger().warn("No response from /home_the_robot service.")
            return

        if response.success:
            self.get_logger().info("Robot returned to home position.")
        else:
            self.get_logger().warn(f"Home request failed: {response.message}")

    def _log_requested_transforms(self, label: str) -> None:
        self.get_logger().info(f"Looking up TFs {label}...")
        self._log_transform("map", "base_link")
        self._log_transform("base_link", "camera_link")
        self._log_transform("base_link", "link_grasp_center")

    def _log_transform(self, target_frame: str, source_frame: str) -> None:
        try:
            wait_future = self._tf_buffer.wait_for_transform_async(
                target_frame,
                source_frame,
                Time(),
            )
            rclpy.spin_until_future_complete(self, wait_future, timeout_sec=TF_TIMEOUT_S)
            if not wait_future.done() or wait_future.result() is None:
                self.get_logger().error(
                    f"Timed out waiting for TF target='{target_frame}', source='{source_frame}'."
                )
                return

            transform = self._tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().error(
                f"TF lookup failed for target='{target_frame}', source='{source_frame}': {exc}"
            )
            return

        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp
        self.get_logger().info(
            "TF target='%s' source='%s' at %d.%09d: "
            "translation=(%.4f, %.4f, %.4f), rotation=(%.4f, %.4f, %.4f, %.4f)"
            % (
                target_frame,
                source_frame,
                stamp.sec,
                stamp.nanosec,
                translation.x,
                translation.y,
                translation.z,
                rotation.x,
                rotation.y,
                rotation.z,
                rotation.w,
            )
        )

    def _send_single_point_goal(
        self,
        label: str,
        joint_names: Sequence[str],
        positions: Sequence[float],
    ) -> bool:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory()
        goal.trajectory.joint_names = list(joint_names)

        point = JointTrajectoryPoint()
        point.positions = list(positions)
        point.time_from_start = Duration(seconds=self._point_duration).to_msg()
        goal.trajectory.points = [point]
        goal.goal_time_tolerance = Duration(seconds=1.0).to_msg()

        self.get_logger().info(f"Sending {label} goal with joints: {goal.trajectory.joint_names}")

        send_future = self._action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if goal_handle is None:
            self.get_logger().error(f"{label} goal handle is None.")
            return False
        if not goal_handle.accepted:
            self.get_logger().error(f"{label} goal was rejected.")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result_wrap = result_future.result()

        if result_wrap is None:
            self.get_logger().error(f"{label} result is None.")
            return False

        status = result_wrap.status
        result = result_wrap.result
        if status != 4 or result.error_code != 0:
            self.get_logger().error(
                f"{label} failed. status={status}, error_code={result.error_code}, "
                f"error_string='{result.error_string}'"
            )
            return False

        self.get_logger().info(f"{label} goal succeeded.")
        return True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Send simple FK and IK-style FollowJointTrajectory goals."
    )
    parser.add_argument(
        "--point-duration",
        type=float,
        default=3.0,
        help="Trajectory point duration in seconds (default: 3.0).",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = IKFKTestClient(point_duration_s=args.point_duration)
    try:
        code = node.run()
    finally:
        node.return_robot_home()
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(code)


if __name__ == "__main__":
    main()
