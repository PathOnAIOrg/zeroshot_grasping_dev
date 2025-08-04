# ROS Setup Guide for Enhanced Digital Twin

## 🤖 **ROS Dependencies**

Some components of the enhanced digital twin system may require ROS (Robot Operating System) for advanced robotics functionality. This guide helps you set up ROS properly.

## 🔍 **Check Current ROS Installation**

### 1. Check if ROS is installed
```bash
echo $ROS_DISTRO
```

**Expected Results:**
- If it shows `humble`, `foxy`, `noetic`, etc. → ROS is installed and sourced
- If it prints nothing → ROS is not sourced or not installed

### 2. Check ROS installation directory
```bash
ls /opt/ros/
```

**Expected Results:**
- Shows available ROS distributions (e.g., `humble`, `noetic`)
- If directory doesn't exist → ROS is not installed

## 📦 **Install ROS (if needed)**

### **Ubuntu 22.04 (ROS 2 Humble)**
```bash
# Update system
sudo apt update
sudo apt upgrade

# Install ROS 2 Humble
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-humble-desktop

```
### **Ubuntu 20.04 (ROS Noetic)**
```bash
# Setup sources
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
sudo apt update

# Install ROS
sudo apt install ros-noetic-desktop-full
```

## 🛠️ **Install rosdep**

### **Method 1: Using apt (Recommended for Ubuntu)**
```bash
sudo apt update
sudo apt install python3-rosdep
```

### **Method 2: Using pip**
```bash
pip3 install rosdep
```

## ⚙️ **Initialize rosdep**

### **Initialize rosdep (run only once)**
```bash
sudo rosdep init
rosdep update
```

**If you get an error like "already exists":**
```bash
# Remove existing rosdep data and reinitialize
sudo rm -rf /etc/ros/rosdep/
sudo rosdep init
rosdep update
```

## 🔧 **Configure Shell Environment**

### **For Bash users (~/.bashrc)**
```bash
# Add to ~/.bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc  # For ROS 2 Humble
# OR
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc  # For ROS Noetic

# Reload shell configuration
source ~/.bashrc
```

### **For Zsh users (~/.zshrc)**
```bash
# Add to ~/.zshrc
echo "source /opt/ros/humble/setup.zsh" >> ~/.zshrc    # For ROS 2 Humble
# OR
echo "source /opt/ros/noetic/setup.zsh" >> ~/.zshrc    # For ROS Noetic

# Reload shell configuration
source ~/.zshrc
```

### **For Fish users (~/.config/fish/config.fish)**
```bash
# Add to ~/.config/fish/config.fish
echo "source /opt/ros/humble/setup.fish" >> ~/.config/fish/config.fish  # For ROS 2 Humble
# OR
echo "source /opt/ros/noetic/setup.fish" >> ~/.config/fish/config.fish  # For ROS Noetic
```

## ✅ **Verify Installation**

### **1. Check rosdep installation**
```bash
which rosdep
rosdep --version
```

**Expected Output:**
```
/usr/bin/rosdep
rosdep 0.22.1
```

### **2. Check ROS environment**
```bash
echo $ROS_DISTRO
echo $ROS_VERSION
```

**Expected Output (ROS 2):**
```
humble
2
```

**Expected Output (ROS 1):**
```
noetic
1
```

### **3. Test ROS functionality**

**For ROS 2:**
```bash
ros2 --help
```

**For ROS 1:**
```bash
roscore --help
```

## 🔄 **Install Additional ROS Packages (if needed)**

### **Common packages for robotics:**
```bash
# ROS 2 Humble
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher
sudo apt install ros-humble-rviz2
sudo apt install ros-humble-moveit

# ROS Noetic
sudo apt install ros-noetic-robot-state-publisher
sudo apt install ros-noetic-joint-state-publisher-gui
sudo apt install ros-noetic-rviz
sudo apt install ros-noetic-moveit
```

## 🐍 **Python ROS Dependencies**

### **Install Python packages:**
```bash
# For ROS 2
pip3 install rclpy geometry_msgs sensor_msgs

# For ROS 1
pip3 install rospy geometry_msgs sensor_msgs
```

## 🚨 **Troubleshooting**

### **Issue: "rosdep command not found"**
**Solution:**
```bash
# Reinstall rosdep
sudo apt remove python3-rosdep
sudo apt install python3-rosdep
sudo rosdep init
rosdep update
```

### **Issue: "ROS_DISTRO not set"**
**Solution:**
```bash
# Manually source ROS setup
source /opt/ros/humble/setup.bash  # or your ROS distro
echo $ROS_DISTRO  # Should now show your distro
```

### **Issue: "Cannot locate rosdep definition"**
**Solution:**
```bash
# Update rosdep database
rosdep update
```

### **Issue: Permission denied for rosdep init**
**Solution:**
```bash
# Use sudo for initialization
sudo rosdep init
# Then update without sudo
rosdep update
```

## 🎯 **ROS Integration with Digital Twin**

### **Benefits of ROS Integration:**
- **URDF/XACRO support** - Better robot model management
- **tf2 transforms** - Coordinate frame management
- **ROS topics** - Real-time data streaming
- **MoveIt integration** - Advanced motion planning
- **Visualization** - RViz integration for 3D visualization

### **Optional: Enable ROS features in Digital Twin**
If you want to use ROS features with the digital twin:

```bash
# Install additional dependencies
pip3 install pykdl_utils tf2_ros

# Set environment variable to enable ROS features
export USE_ROS=true
```

## 📋 **Quick Setup Script**

Create this script to automate the setup:

```bash
#!/bin/bash
# save as setup_ros.sh

echo "🤖 Setting up ROS for Digital Twin"

# Check if ROS is already installed
if [ ! -z "$ROS_DISTRO" ]; then
    echo "✅ ROS $ROS_DISTRO already sourced"
else
    echo "⚠️  ROS not detected, checking installation..."
    
    if [ -d "/opt/ros" ]; then
        # ROS installed but not sourced
        AVAILABLE_DISTROS=$(ls /opt/ros/)
        echo "📦 Found ROS distributions: $AVAILABLE_DISTROS"
        
        # Source the newest available distro
        LATEST_DISTRO=$(ls /opt/ros/ | sort -V | tail -n1)
        echo "🔧 Sourcing ROS $LATEST_DISTRO"
        source /opt/ros/$LATEST_DISTRO/setup.bash
        
        # Add to bashrc for future sessions
        echo "source /opt/ros/$LATEST_DISTRO/setup.bash" >> ~/.bashrc
    else
        echo "❌ ROS not installed. Please install ROS first."
        exit 1
    fi
fi

# Install/check rosdep
if ! command -v rosdep &> /dev/null; then
    echo "📦 Installing rosdep..."
    sudo apt update
    sudo apt install -y python3-rosdep
fi

# Initialize rosdep if needed
if [ ! -d "/etc/ros/rosdep" ]; then
    echo "🔧 Initializing rosdep..."
    sudo rosdep init
fi

echo "🔄 Updating rosdep..."
rosdep update

echo "✅ ROS setup complete!"
echo "   ROS_DISTRO: $ROS_DISTRO"
echo "   rosdep version: $(rosdep --version)"
```

Run the script:
```bash
chmod +x setup_ros.sh
./setup_ros.sh
```

## 🎉 **Verification Checklist**

After setup, verify everything works:

- [ ] `echo $ROS_DISTRO` shows your ROS distribution
- [ ] `which rosdep` shows rosdep location
- [ ] `rosdep --version` shows version number
- [ ] `ros2 --help` works (ROS 2) or `roscore --help` works (ROS 1)
- [ ] No errors when running digital twin scripts

## 🔗 **Additional Resources**

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **ROS 1 Documentation**: http://wiki.ros.org/
- **rosdep Documentation**: http://wiki.ros.org/rosdep
- **Digital Twin with ROS**: See project documentation for ROS integration examples

---

**Note:** The enhanced digital twin system works with or without ROS. ROS integration provides additional features but is not required for basic functionality.