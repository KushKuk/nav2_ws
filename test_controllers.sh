#!/bin/bash

echo "=== ROS2 Controller Spawning Test ==="
echo ""

# Source the workspace
source /home/ujjwal/nav2_ws/install/setup.bash

echo "1. Testing controller manager availability..."
timeout 10s ros2 service list | grep -E "controller_manager|spawner" || echo "Controller manager services not found"

echo ""
echo "2. Testing controller configuration..."
if [ -f "/home/ujjwal/nav2_ws/src/drive/models/drive/config/rover_controllers.yaml" ]; then
    echo "✓ Controller config file exists"
    echo "Controllers defined:"
    grep -E "drive_controller|steer_controller" /home/ujjwal/nav2_ws/src/drive/models/drive/config/rover_controllers.yaml
else
    echo "✗ Controller config file not found"
fi

echo ""
echo "3. Testing URDF ros2_control section..."
if grep -q "ros2_control" /home/ujjwal/nav2_ws/src/drive/models/drive/urdf/drive.urdf; then
    echo "✓ URDF contains ros2_control configuration"
    echo "Joints configured for control:"
    grep -A1 "joint name=" /home/ujjwal/nav2_ws/src/drive/models/drive/urdf/drive.urdf | grep -E "joint_wheel_|joint_.*_front|joint_.*_back"
else
    echo "✗ URDF missing ros2_control configuration"
fi

echo ""
echo "4. Launch file syntax check..."
python3 -c "
import sys
sys.path.append('/home/ujjwal/nav2_ws/src/drive/launch')
try:
    from rover_teleop import generate_launch_description as rover_teleop_launch
    from ros2_control import generate_launch_description as ros2_control_launch
    print('✓ Launch files syntax is valid')
except Exception as e:
    print(f'✗ Launch file syntax error: {e}')
"

echo ""
echo "=== Test Complete ==="
echo ""
echo "To test controller spawning:"
echo "1. Launch: ros2 launch drive rover_teleop.launch.py"
echo "2. Check controllers: ros2 control list_controllers"
echo "3. Verify topics: ros2 topic list | grep -E 'drive_controller|steer_controller'"