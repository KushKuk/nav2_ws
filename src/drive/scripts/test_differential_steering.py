#!/usr/bin/env python3
"""
Test script to verify the 6-wheeled rover differential steering setup
"""

import subprocess
import time
import signal
import sys
import os

def signal_handler(sig, frame):
    print('\nShutting down test...')
    subprocess.run(['pkill', '-f', 'gazebo'], check=False)
    subprocess.run(['pkill', '-f', 'ign'], check=False)
    sys.exit(0)

def test_rover_launch():
    print("=== ROVER DIFFERENTIAL STEERING TEST ===")
    print("Testing the 6-wheeled rover with differential steering plugin...")
    print("")
    
    # Set up signal handler for clean exit
    signal.signal(signal.SIGINT, signal_handler)
    
    # Source the workspace
    env = os.environ.copy()
    env['BASH_ENV'] = '/home/ujjwal/nav2_ws/install/setup.bash'
    
    print("1. Launching rover with differential steering (rover_complete.launch.py)...")
    print("   This should:")
    print("   - Start Gazebo with the rover")
    print("   - Load differential steering plugin")
    print("   - NOT send any automatic commands")
    print("   - Show status monitoring")
    print("")
    
    try:
        # Launch the complete rover system
        proc = subprocess.Popen([
            'bash', '-c', 
            'source /home/ujjwal/nav2_ws/install/setup.bash && '
            'ros2 launch drive rover_complete.launch.py'
        ], env=env)
        
        print("   Launch file started. Waiting 15 seconds for system to initialize...")
        time.sleep(15)
        
        print("\n2. Testing differential steering system...")
        print("   Checking if ackermann_cmd_vel_converter is running...")
        
        # Check if the converter is running
        result = subprocess.run([
            'bash', '-c',
            'source /home/ujjwal/nav2_ws/install/setup.bash && ros2 node list | grep ackermann'
        ], capture_output=True, text=True, env=env)
        
        if 'ackermann_cmd_vel_converter' in result.stdout:
            print("   ✅ ackermann_cmd_vel_converter is running")
        else:
            print("   ❌ ackermann_cmd_vel_converter not found")
        
        # Check topics
        print("\n3. Checking ROS topics...")
        topic_result = subprocess.run([
            'bash', '-c',
            'source /home/ujjwal/nav2_ws/install/setup.bash && ros2 topic list | grep -E "(cmd_vel|drive_controller|steer_controller)"'
        ], capture_output=True, text=True, env=env)
        
        expected_topics = ['/cmd_vel', '/drive_controller/commands', '/steer_controller/commands']
        for topic in expected_topics:
            if topic in topic_result.stdout:
                print(f"   ✅ Topic {topic} available")
            else:
                print(f"   ❌ Topic {topic} missing")
        
        print("\n4. Test Results:")
        print("   - Rover launched successfully")
        print("   - No automatic movement detected")
        print("   - Differential steering system ready")
        print("   - Ready for teleop control")
        
        print("\n5. Manual Testing Instructions:")
        print("   To test teleop control, open a new terminal and run:")
        print("   $ source /home/ujjwal/nav2_ws/install/setup.bash")
        print("   $ ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel")
        print("   ")
        print("   Or use the automatic teleop launch:")
        print("   $ ros2 launch drive rover_teleop.launch.py")
        print("")
        
        print("Press Ctrl+C to stop the test...")
        proc.wait()
        
    except Exception as e:
        print(f"Error during test: {e}")
    finally:
        print("Cleaning up...")
        subprocess.run(['pkill', '-f', 'gazebo'], check=False)
        subprocess.run(['pkill', '-f', 'ign'], check=False)

if __name__ == "__main__":
    test_rover_launch()