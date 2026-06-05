#!/bin/bash

echo "=========================================="
echo "ROS2 Control Setup Verification"
echo "=========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the container
if [ ! -f /opt/ros/humble/setup.bash ]; then
    echo -e "${RED}❌ Error: This script should be run inside the Docker container${NC}"
    echo "Run: docker compose exec ros2_gazebo bash"
    exit 1
fi

# Source ROS2
source /opt/ros/humble/setup.bash

echo "1. Checking ROS2 Control Packages..."
echo "-----------------------------------"
PACKAGES=$(ros2 pkg list | grep -E 'control|controller' | wc -l)
if [ $PACKAGES -gt 0 ]; then
    echo -e "${GREEN}✅ Found $PACKAGES ROS2 control packages${NC}"
    ros2 pkg list | grep -E 'control|controller' | head -10
else
    echo -e "${RED}❌ No ROS2 control packages found${NC}"
fi
echo ""

echo "2. Checking Executables..."
echo "-------------------------"
EXECUTABLES_OK=0

if ros2 run controller_manager --help > /dev/null 2>&1; then
    echo -e "${GREEN}✅ controller_manager found${NC}"
    EXECUTABLES_OK=$((EXECUTABLES_OK + 1))
else
    echo -e "${RED}❌ controller_manager not found${NC}"
fi

if ros2 run joint_state_broadcaster --help > /dev/null 2>&1; then
    echo -e "${GREEN}✅ joint_state_broadcaster found${NC}"
    EXECUTABLES_OK=$((EXECUTABLES_OK + 1))
else
    echo -e "${RED}❌ joint_state_broadcaster not found${NC}"
fi

if ros2 run diff_drive_controller --help > /dev/null 2>&1; then
    echo -e "${GREEN}✅ diff_drive_controller found${NC}"
    EXECUTABLES_OK=$((EXECUTABLES_OK + 1))
else
    echo -e "${RED}❌ diff_drive_controller not found${NC}"
fi
echo ""

echo "3. Checking Gazebo..."
echo "-------------------"
if command -v ign > /dev/null 2>&1; then
    GZ_VERSION=$(ign gazebo --version 2>&1 | head -1)
    echo -e "${GREEN}✅ Gazebo found: $GZ_VERSION${NC}"
else
    echo -e "${RED}❌ Gazebo (ign) not found${NC}"
fi
echo ""

echo "4. Checking Robot Model Files..."
echo "--------------------------------"
SDF_COUNT=0
if [ -f /ros2_ws/exploration/romi_robot_demos/qt_cpp/romi_urdf/romi_world.sdf ]; then
    echo -e "${GREEN}✅ romi_world.sdf found${NC}"
    SDF_COUNT=$((SDF_COUNT + 1))
else
    echo -e "${YELLOW}⚠️  romi_world.sdf not found${NC}"
fi

if [ -f /ros2_ws/exploration/romi_robot_demos/qt_cpp/romi_urdf/romi_world_ros2_control.sdf ]; then
    echo -e "${GREEN}✅ romi_world_ros2_control.sdf found${NC}"
    SDF_COUNT=$((SDF_COUNT + 1))
else
    echo -e "${YELLOW}⚠️  romi_world_ros2_control.sdf not found${NC}"
fi

if [ -f /ros2_ws/exploration/romi_robot_demos/qt_cpp/romi_urdf/romi_controllers.yaml ]; then
    echo -e "${GREEN}✅ romi_controllers.yaml found${NC}"
    SDF_COUNT=$((SDF_COUNT + 1))
else
    echo -e "${YELLOW}⚠️  romi_controllers.yaml not found${NC}"
fi
echo ""

echo "5. Checking ROS2-Gazebo Bridge..."
echo "---------------------------------"
if ros2 pkg list | grep -q ros_gz_bridge; then
    echo -e "${GREEN}✅ ros_gz_bridge package found${NC}"
else
    echo -e "${RED}❌ ros_gz_bridge package not found${NC}"
fi

if ros2 pkg list | grep -q ros_gz_sim; then
    echo -e "${GREEN}✅ ros_gz_sim package found${NC}"
else
    echo -e "${RED}❌ ros_gz_sim package not found${NC}"
fi
echo ""

echo "6. Checking Environment Variables..."
echo "------------------------------------"
if [ -n "$ROS_DOMAIN_ID" ]; then
    echo -e "${GREEN}✅ ROS_DOMAIN_ID=$ROS_DOMAIN_ID${NC}"
else
    echo -e "${YELLOW}⚠️  ROS_DOMAIN_ID not set${NC}"
fi

if [ -n "$GZ_VERSION" ]; then
    echo -e "${GREEN}✅ GZ_VERSION=$GZ_VERSION${NC}"
else
    echo -e "${YELLOW}⚠️  GZ_VERSION not set${NC}"
fi
echo ""

echo "=========================================="
echo "Summary"
echo "=========================================="

if [ $EXECUTABLES_OK -eq 3 ] && [ $PACKAGES -gt 0 ]; then
    echo -e "${GREEN}✅ ROS2 Control setup appears to be working!${NC}"
    echo ""
    echo "Next steps:"
    echo "1. Launch Gazebo: ign gazebo /ros2_ws/exploration/romi_robot_demos/qt_cpp/romi_urdf/romi_world.sdf"
    echo "2. Test control: ign topic -t /model/romi/cmd_vel -m gz.msgs.Twist -p 'linear: {x: 0.2}'"
    exit 0
else
    echo -e "${YELLOW}⚠️  Some components may be missing${NC}"
    echo "Check the output above for details"
    exit 1
fi

