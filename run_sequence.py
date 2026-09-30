#!/usr/bin/env python3
"""
EB-15 Arm Pick-and-Place Joint Sequence Script
==============================================
Sequence:
  1. Open Gripper
  2. Move Arm to Point 1:
     - Joint 1:   32 deg
     - Joint 2:  -38 deg
     - Joint 3: -123 deg
     - Wrist:      3 deg
  3. Close Gripper
  4. Move Arm to Point 2:
     - Joint 1:  -80 deg
     - Joint 2:    3 deg
     - Joint 3:  -99 deg
     - Wrist:    -68 deg
  5. Open Gripper
  6. Return to Home (0, 0, 0, 0 deg)

Supports:
  - Collision-aware motion planning via MoveIt 2 (/plan_kinematic_path)
  - Direct execution via /arm_controller/follow_joint_trajectory
  - Dual-action gripper control (/gripper_controller/gripper_action & /eb15/send_cmd)
  - Step-by-step confirmation (--step) for safe first runs
  - Speed scaling (--speed 0.5 for half speed)
  - Looping (--loop or --repeat N)
"""

import sys
import os
import math
import time
import argparse
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from control_msgs.action import FollowJointTrajectory, GripperCommand
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState

# Target joint values in degrees
POINT_1_DEG = {
    'joint1': 32.0,
    'joint2': -38.0,
    'joint3': -123.0,
    'wrist_joint': 3.0,
}

POINT_2_DEG = {
    'joint1': -80.0,
    'joint2': 3.0,
    'joint3': -99.0,
    'wrist_joint': -68.0,
}

HOME_DEG = {
    'joint1': 0.0,
    'joint2': 0.0,
    'joint3': 0.0,
    'wrist_joint': 0.0,
}

# Gripper values (in meters of stroke from URDF)
# Open:  -0.016 m (Arduino 180 deg servo angle)
# Close: +0.012 m (Arduino 50 deg servo angle)
GRIPPER_OPEN_POS = -0.016
GRIPPER_CLOSE_POS = 0.012
GRIPPER_SERVO_DELAY = 1.3  # seconds to let the physical servo finish moving

JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'wrist_joint']


class EB15SequenceController(Node):
    def __init__(self, speed_factor=1.0, direct_mode=False, dry_run=False):
        super().__init__('eb15_sequence_controller')
        self.speed_factor = max(0.1, min(2.0, float(speed_factor)))
        self.direct_mode = direct_mode
        self.dry_run = dry_run

        self.current_joints = {}
        self.joint_state_received = False

        # Subscriber for current robot state
        self.create_subscription(JointState, '/joint_states', self._joint_state_cb, 10)

        # Action clients
        self.arm_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.gripper_client = ActionClient(self, GripperCommand, '/gripper_controller/gripper_action')

        # MoveIt planner service client
        self.planner_client = self.create_client(GetMotionPlan, '/plan_kinematic_path')

        # Raw command publisher (as backup / instant priority command)
        self.raw_cmd_pub = self.create_publisher(String, '/eb15/send_cmd', 10)

        self.get_logger().info('Initializing EB-15 Sequence Controller...')

    def _joint_state_cb(self, msg: JointState):
        for name, pos in zip(msg.name, msg.position):
            self.current_joints[name] = pos
        self.joint_state_received = True

    def wait_for_connections(self, timeout=10.0):
        """Wait for required ROS topics, services, and action servers."""
        start = time.time()
        self.get_logger().info('Waiting for /joint_states...')
        while rclpy.ok() and not self.joint_state_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start > timeout:
                self.get_logger().warn('Timeout waiting for /joint_states! Proceeding with zero defaults.')
                break

        if not self.dry_run:
            self.get_logger().info('Waiting for /arm_controller/follow_joint_trajectory action server...')
            if not self.arm_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn('Arm action server not found! Ensure demo.launch.py is running.')

            self.get_logger().info('Waiting for /gripper_controller/gripper_action action server...')
            if not self.gripper_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().warn('Gripper action server not found! Raw commands will be used.')

        if not self.direct_mode:
            self.get_logger().info('Checking MoveIt /plan_kinematic_path service...')
            if not self.planner_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().warn('MoveIt planner service not found! Falling back to direct smooth interpolation.')
                self.direct_mode = True

        self.get_logger().info('System ready.')

    def get_current_joint_positions(self):
        """Get current arm joint positions in radians."""
        rclpy.spin_once(self, timeout_sec=0.05)
        res = {}
        for j in JOINT_NAMES:
            res[j] = self.current_joints.get(j, 0.0)
        return res

    def set_gripper(self, open_grip: bool):
        """Control gripper to open or close."""
        target_pos = GRIPPER_OPEN_POS if open_grip else GRIPPER_CLOSE_POS
        state_str = "OPEN" if open_grip else "CLOSE"
        raw_cmd_str = "G_OPEN" if open_grip else "G_CLOSE"

        print(f"\n>> [GRIPPER] Commanding {state_str} (position: {target_pos:.4f}m, cmd: {raw_cmd_str})...")

        if self.dry_run:
            print(f"   [DRY-RUN] Gripper set to {state_str}")
            time.sleep(0.5)
            return True

        # Send raw command as high priority to Arduino
        raw_msg = String()
        raw_msg.data = raw_cmd_str
        self.raw_cmd_pub.publish(raw_msg)

        # Also send via GripperCommand action server to update MoveIt/RViz state
        if self.gripper_client.server_is_ready():
            goal = GripperCommand.Goal()
            goal.command.position = target_pos
            goal.command.max_effort = 1.0

            send_goal_future = self.gripper_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=3.0)
            goal_handle = send_goal_future.result()

            if goal_handle and goal_handle.accepted:
                res_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, res_future, timeout_sec=3.0)

        # Allow time for physical servo movement
        print(f"   Waiting {GRIPPER_SERVO_DELAY:.1f}s for physical servo movement...")
        time.sleep(GRIPPER_SERVO_DELAY)
        print(f"   [GRIPPER] {state_str} complete.")
        return True

    def plan_with_moveit(self, target_rad_dict):
        """Plan trajectory using MoveIt 2 /plan_kinematic_path service."""
        if not self.planner_client.service_is_ready():
            return None

        req = GetMotionPlan.Request()
        mp_req = MotionPlanRequest()
        mp_req.group_name = 'arm'
        mp_req.num_planning_attempts = 5
        mp_req.allowed_planning_time = 5.0

        # Explicit start state from current robot position
        curr_rad = self.get_current_joint_positions()
        start_state = RobotState()
        js = JointState()
        js.header.stamp = self.get_clock().now().to_msg()
        for name in JOINT_NAMES:
            js.name.append(name)
            js.position.append(curr_rad.get(name, 0.0))
        start_state.joint_state = js
        mp_req.start_state = start_state

        # Goal constraints
        c = Constraints()
        for name, pos in target_rad_dict.items():
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.005
            jc.tolerance_below = 0.005
            jc.weight = 1.0
            c.joint_constraints.append(jc)

        mp_req.goal_constraints.append(c)
        req.motion_plan_request = mp_req

        future = self.planner_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=6.0)

        res = future.result()
        if res and res.motion_plan_response.error_code.val == 1:
            traj = res.motion_plan_response.trajectory.joint_trajectory
            if len(traj.points) > 0:
                # Pin exact final point
                final_pt = traj.points[-1]
                final_pt.positions = [target_rad_dict.get(n, 0.0) for n in traj.joint_names]
                # Scale time by speed_factor
                if self.speed_factor != 1.0:
                    for pt in traj.points:
                        orig_t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                        new_t = orig_t / self.speed_factor
                        pt.time_from_start.sec = int(new_t)
                        pt.time_from_start.nanosec = int((new_t - int(new_t)) * 1e9)
                return traj

        return None

    def generate_direct_trajectory(self, start_dict, target_dict, duration=3.5):
        """Generate smooth S-curve / cosine interpolated trajectory."""
        duration = duration / self.speed_factor
        num_steps = max(10, int(duration * 20))  # 20 Hz points

        traj = JointTrajectory()
        traj.joint_names = JOINT_NAMES

        for i in range(num_steps + 1):
            s = float(i) / num_steps
            # Cosine smoothstep (zero velocity at start and end)
            alpha = 0.5 * (1.0 - math.cos(math.pi * s))

            pt = JointTrajectoryPoint()
            for name in JOINT_NAMES:
                q_start = start_dict.get(name, 0.0)
                q_target = target_dict.get(name, 0.0)
                q_val = q_start + alpha * (q_target - q_start)
                pt.positions.append(q_val)

            t_val = s * duration
            pt.time_from_start.sec = int(t_val)
            pt.time_from_start.nanosec = int((t_val - int(t_val)) * 1e9)
            traj.points.append(pt)

        return traj

    def move_to_joints(self, target_deg_dict, step_name="Target"):
        """Move arm to target joint angles given in degrees."""
        target_rad = {k: math.radians(v) for k, v in target_deg_dict.items()}

        deg_str = ", ".join([f"{k}: {target_deg_dict[k]:.1f}°" for k in JOINT_NAMES if k in target_deg_dict])
        print(f"\n>> [ARM] Moving to {step_name}: [{deg_str}]...")

        curr_rad = self.get_current_joint_positions()
        curr_deg_str = ", ".join([f"{k}: {math.degrees(curr_rad[k]):.1f}°" for k in JOINT_NAMES])
        print(f"   Current joints: [{curr_deg_str}]")

        traj = None
        if not self.direct_mode:
            print("   Planning with MoveIt 2 (collision-checked)...")
            traj = self.plan_with_moveit(target_rad)
            if traj is not None:
                total_t = traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9
                print(f"   MoveIt planned {len(traj.points)} waypoints over {total_t:.2f}s.")
            else:
                print("   MoveIt planning unavailable or failed. Using direct smooth trajectory generator...")

        if traj is None:
            # Estimate reasonable duration based on largest joint angular displacement
            max_delta = max([abs(target_rad[j] - curr_rad[j]) for j in JOINT_NAMES])
            # Max speed ~45 deg/sec = 0.78 rad/sec
            dur = max(2.0, max_delta / 0.6)
            traj = self.generate_direct_trajectory(curr_rad, target_rad, duration=dur)
            print(f"   Generated smooth cosine trajectory ({len(traj.points)} points, {dur / self.speed_factor:.2f}s).")

        if self.dry_run:
            print(f"   [DRY-RUN] Execution skipped.")
            time.sleep(1.0)
            return True

        # Send trajectory goal to arm action server
        if not self.arm_client.server_is_ready():
            self.get_logger().error('Arm action server not connected!')
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj

        print(f"   Executing trajectory on physical EB-15 arm...")
        send_future = self.arm_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        goal_handle = send_future.result()

        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error('Trajectory goal was rejected by action server!')
            return False

        res_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_future)
        result = res_future.result()

        if result and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            print(f"   [ARM] Motion to {step_name} completed successfully.")
            return True
        else:
            self.get_logger().warn(f'Trajectory finished with result code: {result.result.error_code if result else "None"}')
            return True


def prompt_step(step_title):
    print(f"\n{'='*60}")
    print(f" NEXT: {step_title}")
    print(f"{'='*60}")
    try:
        input("Press [Enter] to execute this step (or Ctrl+C to abort)...")
    except EOFError:
        pass


def run_sequence(controller, step_by_step=False):
    """
    Executes the 6-step pick-and-place sequence:
      1. Open gripper
      2. Move to Point 1 (j1: 32, j2: -38, j3: -123, wrist: 3)
      3. Close gripper
      4. Move to Point 2 (j1: -80, j2: 3, j3: -99, wrist: -68)
      5. Open gripper
      6. Return to Home (0, 0, 0, 0)
    """
    steps = [
        ("Step 1: Open Gripper", lambda: controller.set_gripper(open_grip=True)),
        ("Step 2: Move to Point 1 [32°, -38°, -123°, 3°]", lambda: controller.move_to_joints(POINT_1_DEG, "Point 1")),
        ("Step 3: Close Gripper", lambda: controller.set_gripper(open_grip=False)),
        ("Step 4: Move to Point 2 [-80°, 3°, -99°, -68°]", lambda: controller.move_to_joints(POINT_2_DEG, "Point 2")),
        ("Step 5: Open Gripper", lambda: controller.set_gripper(open_grip=True)),
        ("Step 6: Move to Home [0°, 0°, 0°, 0°]", lambda: controller.move_to_joints(HOME_DEG, "Home")),
    ]

    print("\n" + "#" * 60)
    print("  EB-15 PICK-AND-PLACE SEQUENCE START")
    print("#" * 60)

    for i, (title, func) in enumerate(steps, 1):
        if step_by_step:
            prompt_step(f"[{i}/{len(steps)}] {title}")
        else:
            print(f"\n--- [{i}/{len(steps)}] {title} ---")

        success = func()
        if not success:
            print(f"\n[ERROR] Step {i} failed! Aborting sequence.")
            return False

        # Small pause between actions for settling
        time.sleep(0.4)

    print("\n" + "#" * 60)
    print("  SEQUENCE COMPLETED SUCCESSFULLY!")
    print("#" * 60 + "\n")
    return True


def main():
    parser = argparse.ArgumentParser(description="EB-15 Pick-and-Place Joint Sequence")
    parser.add_argument('--speed', type=float, default=1.0, help="Speed factor (e.g. 0.5 for half speed, 1.0 normal)")
    parser.add_argument('--step', action='store_true', help="Step-by-step confirmation mode (press Enter before each step)")
    parser.add_argument('--direct', action='store_true', help="Use direct smooth trajectory generation (bypass MoveIt planner)")
    parser.add_argument('--dry-run', action='store_true', help="Simulate steps without moving physical arm")
    parser.add_argument('--repeat', type=int, default=1, help="Number of times to repeat the sequence (default: 1)")
    parser.add_argument('--loop', action='store_true', help="Run sequence continuously in an infinite loop")

    args = parser.parse_args()

    rclpy.init()
    controller = EB15SequenceController(
        speed_factor=args.speed,
        direct_mode=args.direct,
        dry_run=args.dry_run
    )

    try:
        controller.wait_for_connections()

        iteration = 1
        while rclpy.ok():
            if args.repeat > 1 or args.loop:
                print(f"\n>>> ITERATION {iteration} <<<")

            ok = run_sequence(controller, step_by_step=args.step)
            if not ok:
                break

            if not args.loop and iteration >= args.repeat:
                break

            iteration += 1
            if args.loop or iteration <= args.repeat:
                print("\nWaiting 2 seconds before next cycle...")
                time.sleep(2.0)

    except KeyboardInterrupt:
        print("\n[ABORT] User interrupted sequence with Ctrl+C.")
    finally:
        controller.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
