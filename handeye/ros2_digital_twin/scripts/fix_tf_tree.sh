#!/bin/bash
# Fix the disconnected TF tree by connecting camera to robot

echo "=========================================="
echo "🔧 Fixing TF Tree Connection"
echo "=========================================="
echo ""
echo "Current situation:"
echo "  - Robot TF tree: base → ... → gripper"
echo "  - Camera TF tree: camera_link → camera_*_frame (disconnected)"
echo ""
echo "Solution: Connect gripper → camera_link"
echo ""

# Check which gripper frame name is used
echo "Checking available TF frames..."
ros2 topic echo /tf --once 2>/dev/null | grep frame_id | grep -E "(gripper|wrist)" | head -5

echo ""
echo "=========================================="
echo "Trying different connection points..."
echo "=========================================="

# Try gripper first
echo "1. Trying: gripper → camera_link"
python3 ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin/connect_camera_to_robot.py --ros-args -p gripper_frame:=gripper &
PID1=$!
sleep 2

# Check if it worked
if ros2 run tf2_ros tf2_echo base camera_link --timeout 2 2>/dev/null | grep -q "Translation"; then
    echo "✅ SUCCESS! Camera connected via 'gripper'"
else
    kill $PID1 2>/dev/null
    
    # Try link_gripper
    echo "2. Trying: link_gripper → camera_link"
    python3 ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin/connect_camera_to_robot.py --ros-args -p gripper_frame:=link_gripper &
    PID2=$!
    sleep 2
    
    if ros2 run tf2_ros tf2_echo base camera_link --timeout 2 2>/dev/null | grep -q "Translation"; then
        echo "✅ SUCCESS! Camera connected via 'link_gripper'"
    else
        kill $PID2 2>/dev/null
        
        # Try wrist_roll
        echo "3. Trying: wrist_roll → camera_link"
        python3 ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin/connect_camera_to_robot.py --ros-args -p gripper_frame:=wrist_roll &
        PID3=$!
        sleep 2
        
        if ros2 run tf2_ros tf2_echo base camera_link --timeout 2 2>/dev/null | grep -q "Translation"; then
            echo "✅ SUCCESS! Camera connected via 'wrist_roll'"
        else
            kill $PID3 2>/dev/null
            
            # Try link5
            echo "4. Trying: link5 → camera_link"
            python3 ~/Documents/Github/opensource_dev/handeye/ros2_digital_twin/connect_camera_to_robot.py --ros-args -p gripper_frame:=link5 &
            PID4=$!
            sleep 2
            
            if ros2 run tf2_ros tf2_echo base camera_link --timeout 2 2>/dev/null | grep -q "Translation"; then
                echo "✅ SUCCESS! Camera connected via 'link5'"
            else
                echo "❌ Could not find correct gripper frame"
                echo "Please check your robot's TF tree and specify manually"
                kill $PID4 2>/dev/null
            fi
        fi
    fi
fi

echo ""
echo "=========================================="
echo "📊 Verification"
echo "=========================================="
echo ""
echo "Checking transform from base to camera..."
ros2 run tf2_ros tf2_echo base camera_link --timeout 2

echo ""
echo "=========================================="
echo "✅ Setup Complete!"
echo "=========================================="
echo ""
echo "Now in RViz2:"
echo "1. Set Fixed Frame to: 'base'"
echo "2. The point cloud should appear at the robot's gripper"
echo "3. You should see all TF frames connected"
echo ""
echo "To verify, run:"
echo "  ros2 run tf2_tools view_frames"
echo "  evince frames.pdf"
echo ""