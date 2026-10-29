#!/usr/bin/env python3
"""
EB-15 Inverse Kinematics (IK) Test Script
==========================================
Tests Inverse Kinematics for a given 3D End-Effector / TCP target (X, Y, Z).
Validates analytical IK against Forward Kinematics (FK) and MoveIt 2's /compute_ik service.

Usage:
  python3 src/eb15_driver/eb15_driver/test_ik.py [X] [Y] [Z] [--roll ROLL_RAD] [--publish]

Examples:
  python3 src/eb15_driver/eb15_driver/test_ik.py 0.200 0.150 0.250
  python3 src/eb15_driver/eb15_driver/test_ik.py 0.250 0.000 0.200 --publish
"""

import sys
import math
import argparse

# EB-15 Physical Dimension Constants (Meters)
D_BASE = 0.0738       # Base ground to Joint 2 (Shoulder) axis (d1=0.0308 + h1=0.0430)
A2 = 0.2280           # Upper Arm length (Shoulder axis to Elbow axis)
A3 = 0.2657           # Forearm + Wrist Flange (L3=0.1780 + L4=0.0877)
L_TCP = 0.0885        # Tool0 flange to fingertip grasp point

# Padded Joint Limits (URDF / Firmware)
LIMITS = {
    'joint1': (-3.1416, 3.1416),
    'joint2': (-1.5708, 1.5708),
    'joint3': (-2.7000, 2.7000),
    'wrist_joint': (-1.5708, 1.5708)
}

def solve_analytical_ik(x, y, z, wrist_roll=0.0, elbow_config='up'):
    """
    Computes closed-form Analytical IK for EB-15 arm chain to reach target (x, y, z) at tool0.
    """
    # 1. Base Yaw (q1)
    q1 = math.atan2(y, x)
    
    # 2. Cylindrical planar coordinates
    r = math.sqrt(x**2 + y**2)
    s = z - D_BASE
    D = math.sqrt(r**2 + s**2)
    
    # Reachability check
    min_reach = abs(A2 - A3)
    max_reach = A2 + A3
    if D > max_reach:
        raise ValueError(f"Target is OUT OF REACH! Distance {D*1000:.1f} mm exceeds max reach {max_reach*1000:.1f} mm.")
    if D < min_reach:
        raise ValueError(f"Target is in the INNER DEAD ZONE! Distance {D*1000:.1f} mm is within min reach {min_reach*1000:.1f} mm.")
        
    # 3. Elbow Pitch (q3) via Law of Cosines
    cos_q3 = (D**2 - A2**2 - A3**2) / (2.0 * A2 * A3)
    cos_q3 = max(-1.0, min(1.0, cos_q3))
    
    if elbow_config == 'up':
        sin_q3 = -math.sqrt(1.0 - cos_q3**2)
    else:
        sin_q3 = math.sqrt(1.0 - cos_q3**2)
        
    q3_dh = math.atan2(sin_q3, cos_q3)
    
    # 4. Shoulder Pitch (q2)
    beta = math.atan2(s, r)
    psi = math.atan2(A3 * sin_q3, A2 + A3 * cos_q3)
    q2_dh = beta - psi
    
    # Convert DH frame angles to URDF standard zero reference:
    # In URDF: Upright candle posture along +Z is q2 = 0.0 rad, q3 = 0.0 rad
    # Rotation about +Y tilts the arm forward (+X).
    q1_urdf = q1
    q2_urdf = (math.pi / 2.0) - q2_dh
    q3_urdf = -q3_dh
    q_wrist_urdf = wrist_roll - q1
    
    # Clamp wrist to [-pi/2, +pi/2]
    q_wrist_urdf = math.atan2(math.sin(q_wrist_urdf), math.cos(q_wrist_urdf))
    q_wrist_urdf = max(LIMITS['wrist_joint'][0], min(LIMITS['wrist_joint'][1], q_wrist_urdf))
    
    return {
        'joint1': q1_urdf,
        'joint2': q2_urdf,
        'joint3': q3_urdf,
        'wrist_joint': q_wrist_urdf
    }

def forward_kinematics(q):
    """
    Computes Forward Kinematics (FK) position (x, y, z) of tool0 given URDF joint angles.
    In URDF frame: Upright candle is Z+, pitch rotations rotate around Y+.
    """
    q1 = q['joint1']
    q2 = q['joint2']
    q3 = q['joint3']
    
    pitch = q2 + q3
    r = A2 * math.sin(q2) + A3 * math.sin(pitch)
    x = r * math.cos(q1)
    y = r * math.sin(q1)
    z = D_BASE + A2 * math.cos(q2) + A3 * math.cos(pitch)
    return x, y, z

def test_moveit_services(target_x, target_y, target_z, q_solution):
    """
    Tests ROS 2 MoveIt /compute_fk and /compute_ik services.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from moveit_msgs.srv import GetPositionFK, GetPositionIK
        from sensor_msgs.msg import JointState
    except ImportError:
        print("[INFO] rclpy / MoveIt messages not available in this Python environment.")
        return

    rclpy.init()
    node = Node('eb15_ik_tester')
    
    try:
        fk_client = node.create_client(GetPositionFK, '/compute_fk')
        ik_client = node.create_client(GetPositionIK, '/compute_ik')
        
        if not fk_client.wait_for_service(timeout_sec=1.0) or not ik_client.wait_for_service(timeout_sec=1.0):
            print("[INFO] MoveIt services (/compute_fk, /compute_ik) are not running (launch demo.launch.py first).")
            return
            
        print("\n--- Verifying with MoveIt 2 Kinematics Services ---")
        
        # 1. Compute FK using MoveIt KDL
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = 'base_link'
        fk_req.fk_link_names = ['tool0']
        
        js = JointState()
        js.name = list(q_solution.keys())
        js.position = list(q_solution.values())
        fk_req.robot_state.joint_state = js
        
        future_fk = fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(node, future_fk, timeout_sec=2.0)
        fk_res = future_fk.result()
        
        if fk_res and fk_res.error_code.val == 1:
            tool0_pose = fk_res.pose_stamped[0].pose
            print(f"MoveIt FK Position: X={tool0_pose.position.x:.4f} m, Y={tool0_pose.position.y:.4f} m, Z={tool0_pose.position.z:.4f} m")
            
            # 2. Compute IK using MoveIt with the exact feasible pose
            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = 'arm'
            ik_req.ik_request.pose_stamped.header.frame_id = 'base_link'
            ik_req.ik_request.pose_stamped.pose = tool0_pose
            ik_req.ik_request.timeout.sec = 1
            
            future_ik = ik_client.call_async(ik_req)
            rclpy.spin_until_future_complete(node, future_ik, timeout_sec=2.0)
            ik_res = future_ik.result()
            
            if ik_res and ik_res.error_code.val == 1:
                print("MoveIt /compute_ik Status: SUCCESS (Error Code: 1)")
            else:
                code = ik_res.error_code.val if ik_res else 'TIMEOUT'
                print(f"MoveIt /compute_ik Status: Returned code {code}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

def publish_to_ros(q):
    """
    Publishes the solved joint angles to /eb15/joint_commands topic.
    """
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import JointState
        
        rclpy.init()
        node = Node('eb15_ik_publisher')
        pub = node.create_publisher(JointState, '/eb15/joint_commands', 10)
        
        msg = JointState()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.name = ['joint1', 'joint2', 'joint3', 'wrist_joint', 'gripper_left_joint']
        msg.position = [q['joint1'], q['joint2'], q['joint3'], q['wrist_joint'], 0.0]
        
        import time
        time.sleep(0.3)
        pub.publish(msg)
        print("[SUCCESS] Published target joints to /eb15/joint_commands topic!")
        node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        print(f"[ERROR] Failed to publish: {e}")

def main():
    parser = argparse.ArgumentParser(description="Test EB-15 Inverse Kinematics (IK) for a fixed TCP position.")
    parser.add_argument('x', type=float, nargs='?', default=0.200, help="Target X coordinate in meters (default: 0.200)")
    parser.add_argument('y', type=float, nargs='?', default=0.150, help="Target Y coordinate in meters (default: 0.150)")
    parser.add_argument('z', type=float, nargs='?', default=0.250, help="Target Z coordinate in meters (default: 0.250)")
    parser.add_argument('--roll', type=float, default=0.0, help="Target wrist roll in radians (default: 0.0)")
    parser.add_argument('--elbow', choices=['up', 'down'], default='up', help="Elbow branch configuration (default: up)")
    parser.add_argument('--publish', action='store_true', help="Publish the solved joint state to /eb15/joint_commands")
    
    args = parser.parse_args()
    
    print("=" * 65)
    print("       EB-15 INVERSE KINEMATICS (IK) TEST BENCH")
    print("=" * 65)
    print(f"Target Tool0 Position: X = {args.x:.4f} m, Y = {args.y:.4f} m, Z = {args.z:.4f} m")
    print(f"Target Wrist Roll:     {args.roll:.4f} rad ({math.degrees(args.roll):.1f}°)")
    print(f"Kinematic Branch:      Elbow-{args.elbow.capitalize()}")
    print("-" * 65)
    
    try:
        q = solve_analytical_ik(args.x, args.y, args.z, wrist_roll=args.roll, elbow_config=args.elbow)
    except ValueError as e:
        print(f"[FAILED] {e}")
        sys.exit(1)
        
    print("Computed Joint Angles:")
    for j_name, val in q.items():
        deg = math.degrees(val)
        low, high = LIMITS[j_name]
        in_bounds = low <= val <= high
        status = "OK" if in_bounds else "EXCEEDS LIMITS!"
        print(f"  {j_name:12s}: {val:+8.4f} rad  ({deg:+7.2f}°)  [{status}]")
        
    # Forward Kinematics Verification
    fk_x, fk_y, fk_z = forward_kinematics(q)
    err = math.sqrt((args.x - fk_x)**2 + (args.y - fk_y)**2 + (args.z - fk_z)**2)
    print("-" * 65)
    print(f"Forward Kinematics Position Check:")
    print(f"  Calculated: X = {fk_x:.4f} m, Y = {fk_y:.4f} m, Z = {fk_z:.4f} m")
    print(f"  Position Error: {err * 1000.0:.6f} mm")
    
    # Arduino Serial Packet Generation
    cmd_str = f"$CMD,{q['joint1']:.4f},{q['joint2']:.4f},{q['joint3']:.4f},{q['wrist_joint']:.4f},0.0000\\n"
    print(f"Arduino Serial Protocol Command:\n  {cmd_str.strip()}")
    print("=" * 65)
    
    # MoveIt 2 Service verification
    test_moveit_services(args.x, args.y, args.z, q)
    
    if args.publish:
        publish_to_ros(q)

if __name__ == '__main__':
    main()
