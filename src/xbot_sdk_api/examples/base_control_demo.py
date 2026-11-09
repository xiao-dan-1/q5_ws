#!/usr/bin/env python3
"""
Base Control Demo
专门演示底盘控制功能的独立示例。
包括前进、后退、转向、组合运动等底盘控制操作。
"""

import rclpy
import sys
import os
import time

# Add the parent directory to the path to import robot_controller
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from robot_controller import RobotController, TrajectoryController


def main():
    """Main function demonstrating base control functionality."""
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create robot controller
        print("Creating robot controller for base control demo...")
        robot = RobotController("base_control_demo")
        
        # Create specialized controllers
        trajectory_controller = TrajectoryController(robot)
        
        print("Robot controller created successfully!")
        
        # Step 1: Start joint service
        print("\n=== Step 1: Starting Joint Service ===")
        if robot.start_joint_service("", False, "pos"):
            print("✓ Joint service started successfully")
        else:
            print("✗ Failed to start joint service")
            return
        
        # Step 2: Initialize joints (this takes about 20 seconds)
        print("\n=== Step 2: Initializing Joints ===")
        print("This process takes about 20 seconds...")
        if robot.initialize_joints():
            print("✓ Joints initialized successfully")
        else:
            print("✗ Failed to initialize joints")
            return
        
        # Step 3: Set to zero position
        print("\n=== Step 3: Setting Zero Position ===")
        print("This process takes about 2 seconds...")
        if trajectory_controller.set_zero_position(duration=4.0):
            print("✓ Zero position set successfully")
        else:
            print("✗ Failed to set zero position")
            return
        
        # Step 4: Set lift up position (to prepare for movement)
        print("\n=== Step 4: Setting Lift Up Position ===")
        if trajectory_controller.set_lift_up_position(duration=3.0):
            print("✓ Lift up position set successfully")
        else:
            print("✗ Failed to set lift up position")
        
        # Step 5: Basic Movement Tests
        print("\n=== Step 5: Basic Movement Tests ===")
        
        # Test 1: Forward movement
        print("\n--- Test 1: Forward Movement ---")
        print("Moving forward at 0.2 m/s for 3 seconds...")
        if robot.control_base_drive(linear_x=0.2, angular_z=0.0):
            print("✓ Forward movement command sent")
            time.sleep(3.0)
        else:
            print("✗ Failed to send forward movement command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 2: Backward movement
        print("\n--- Test 2: Backward Movement ---")
        print("Moving backward at -0.15 m/s for 3 seconds...")
        if robot.control_base_drive(linear_x=-0.15, angular_z=0.0):
            print("✓ Backward movement command sent")
            time.sleep(3.0)
        else:
            print("✗ Failed to send backward movement command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 3: Left turn
        print("\n--- Test 3: Left Turn ---")
        print("Turning left at 0.5 rad/s for 2 seconds...")
        if robot.control_base_drive(linear_x=0.0, angular_z=0.5):
            print("✓ Left turn command sent")
            time.sleep(2.0)
        else:
            print("✗ Failed to send left turn command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 4: Right turn
        print("\n--- Test 4: Right Turn ---")
        print("Turning right at -0.5 rad/s for 2 seconds...")
        if robot.control_base_drive(linear_x=0.0, angular_z=-0.5):
            print("✓ Right turn command sent")
            time.sleep(2.0)
        else:
            print("✗ Failed to send right turn command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Step 6: Combined Movement Tests
        print("\n=== Step 6: Combined Movement Tests ===")
        
        # Test 5: Forward + Left turn
        print("\n--- Test 5: Forward + Left Turn ---")
        print("Moving forward while turning left for 3 seconds...")
        if robot.control_base_drive(linear_x=0.1, angular_z=0.3):
            print("✓ Combined movement command sent")
            time.sleep(3.0)
        else:
            print("✗ Failed to send combined movement command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 6: Forward + Right turn
        print("\n--- Test 6: Forward + Right Turn ---")
        print("Moving forward while turning right for 3 seconds...")
        if robot.control_base_drive(linear_x=0.1, angular_z=-0.3):
            print("✓ Combined movement command sent")
            time.sleep(3.0)
        else:
            print("✗ Failed to send combined movement command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Step 7: Speed Variation Tests
        print("\n=== Step 7: Speed Variation Tests ===")
        
        # Test 7: Slow forward
        print("\n--- Test 7: Slow Forward Movement ---")
        print("Moving forward slowly at 0.05 m/s for 2 seconds...")
        if robot.control_base_drive(linear_x=0.05, angular_z=0.0):
            print("✓ Slow forward movement command sent")
            time.sleep(2.0)
        else:
            print("✗ Failed to send slow forward movement command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 8: Fast forward
        print("\n--- Test 8: Fast Forward Movement ---")
        print("Moving forward quickly at 0.3 m/s for 2 seconds...")
        if robot.control_base_drive(linear_x=0.3, angular_z=0.0):
            print("✓ Fast forward movement command sent")
            time.sleep(2.0)
        else:
            print("✗ Failed to send fast forward movement command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 9: Slow turn
        print("\n--- Test 9: Slow Turn ---")
        print("Turning slowly at 0.2 rad/s for 3 seconds...")
        if robot.control_base_drive(linear_x=0.0, angular_z=0.2):
            print("✓ Slow turn command sent")
            time.sleep(3.0)
        else:
            print("✗ Failed to send slow turn command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Test 10: Fast turn
        print("\n--- Test 10: Fast Turn ---")
        print("Turning quickly at 0.8 rad/s for 1 second...")
        if robot.control_base_drive(linear_x=0.0, angular_z=0.8):
            print("✓ Fast turn command sent")
            time.sleep(1.0)
        else:
            print("✗ Failed to send fast turn command")
        
        # Stop
        print("Stopping...")
        robot.stop_base_drive()
        time.sleep(1.0)
        
        # Step 8: Complex Movement Pattern
        print("\n=== Step 8: Complex Movement Pattern ===")
        print("Executing a complex movement pattern...")
        
        # Pattern: Forward -> Turn -> Forward -> Turn -> Stop
        movements = [
            ("Forward", 0.15, 0.0, 2.0),
            ("Left Turn", 0.0, 0.4, 1.5),
            ("Forward", 0.15, 0.0, 2.0),
            ("Right Turn", 0.0, -0.4, 1.5),
            ("Forward", 0.1, 0.0, 1.5),
            ("Stop", 0.0, 0.0, 0.5)
        ]
        
        for movement_name, linear_x, angular_z, duration in movements:
            print(f"  {movement_name}: linear_x={linear_x}, angular_z={angular_z}")
            if robot.control_base_drive(linear_x=linear_x, angular_z=angular_z):
                print(f"  ✓ {movement_name} command sent")
                time.sleep(duration)
            else:
                print(f"  ✗ Failed to send {movement_name} command")
        
        # Final stop
        print("\nFinal stop...")
        robot.stop_base_drive()
        
        # Step 9: Final status
        print("\n=== Step 9: Final Status ===")
        final_joints_info = robot.get_all_joints_info()
        if final_joints_info:
            print("Final joint positions:")
            for joint in final_joints_info[:5]:  # Show first 5 joints
                print(f"  {joint['name']}: {joint['position']:.3f} rad")
            if len(final_joints_info) > 5:
                print(f"  ... and {len(final_joints_info) - 5} more joints")
        
        print("\n=== Base Control Demo Completed Successfully! ===")
        print("The robot has completed all base movement tests.")
        
    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
        # Emergency stop
        try:
            if 'robot' in locals():
                robot.stop_base_drive()
                print("Emergency stop executed")
        except:
            pass
    except Exception as e:
        print(f"Error during demo: {str(e)}")
        # Emergency stop
        try:
            if 'robot' in locals():
                robot.stop_base_drive()
                print("Emergency stop executed")
        except:
            pass
    finally:
        # Cleanup
        try:
            if 'robot' in locals():
                robot.stop_base_drive()  # Ensure robot is stopped
                robot.shutdown()
        except:
            pass
        rclpy.shutdown()


if __name__ == "__main__":
    main()
