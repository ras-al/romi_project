# SplaTAM ROS 2 Live Bridge: Complete Setup & Execution Guide

This comprehensive guide is designed to help set up a brand new GPU PC from scratch to run real-time 3D Gaussian Splatting using [SplaTAM](https://github.com/spla-tam/SplaTAM), and connect it to a Gazebo simulation running on a separate laptop.

---

## 🖥️ Phase 1: GPU PC System Requirements & ROS 2 Setup

The GPU PC needs to run ROS 2 to communicate with the laptop. We assume the GPU PC is running **Ubuntu 22.04** (required for ROS 2 Humble).

### Step 1: Install ROS 2 Humble (If not already installed)
Open a terminal on the GPU PC and run the official ROS 2 installation steps:

```bash
locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

sudo apt install software-properties-common
sudo add-apt-repository universe

sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install ros-humble-desktop
sudo apt install ros-dev-tools
```

### Step 2: Install ROS 2 CV Bridge
The PyTorch script needs `cv_bridge` to convert ROS images to OpenCV formats:

```bash
sudo apt update
sudo apt install ros-humble-cv-bridge
```

---

## 🛠️ Phase 2: Cloning the Repositories

You need both your `romi_project` (which contains our custom bridge scripts) and the `SplaTAM` repository on the GPU PC.

### Step 1: Clone your Romi Project
Assuming your code is hosted on GitHub or another Git service:
```bash
cd ~

git clone https://github.com/ras-al/romi_project.git
```

### Step 2: Clone SplaTAM
```bash
cd ~
git clone https://github.com/spla-tam/SplaTAM.git
cd SplaTAM
```

---

## 🐍 Phase 3: SplaTAM Environment Setup

The GPU machine will handle the heavy PyTorch optimization to construct the 3D Gaussians. 

### Step 1: Install the Conda Environment
Ensure Anaconda or Miniconda is installed. Then create the environment:

```bash
# Create and activate the conda environment
conda create -n splatam python=3.10
conda activate splatam

# Install PyTorch (Update the CUDA version if necessary for your specific GPU)
conda install pytorch torchvision torchaudio pytorch-cuda=11.8 -c pytorch -c nvidia

# Install SplaTAM dependencies
pip install -r requirements.txt
pip install -e .

# Install ROS 2 Python interop packages inside the conda env
pip install rospkg PyYAML
```

### Step 2: Transfer the Bridge Scripts
Now, move the bridge scripts from your `romi_project` into the `SplaTAM` repository so Python can find the `splatam` module:

```bash
# Move the bridge script to the root of SplaTAM
cp ~/romi_project/splatam_workspace/splatam_live_viewer.py ~/SplaTAM/

# Move the configuration file to the configs folder
cp ~/romi_project/splatam_workspace/configs/romi_live.py ~/SplaTAM/configs/
```

---

## 🚀 Phase 4: Execution Guide

To run this pipeline, **both machines must be connected to the same local network** (a wired ethernet connection is highly recommended due to high RGB-D bandwidth).

### 1. Laptop (Physics Simulation)
Start the Gazebo simulation and the autonomy node on your laptop. The ROS 2 network will automatically start broadcasting the depth camera images.

```bash
# Ensure both machines share the exact same Domain ID
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Launch Romi
cd ~/Documents/robotics/romi_project/romi_ws
source install/setup.bash
ros2 launch romi_gazebo romi_control.launch.py explore:=true
```

### 2. GPU PC (Neural Rendering)
Start the visualizer on your GPU machine. The script will intercept the ROS 2 network traffic, initialize the 3D Gaussians, and pop open the SplaTAM interactive viewer.

```bash
# Source ROS 2 base installation
source /opt/ros/humble/setup.bash

# Ensure both machines share the exact same Domain ID
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Navigate to the SplaTAM repository
cd ~/SplaTAM
conda activate splatam

# Launch the live SplaTAM GUI bridge
python splatam_live_viewer.py
```

---

## 🎮 Interacting with the Map
Once the GUI opens on the GPU PC, you will see a colorized 3D point cloud expanding in real time as the robot explores the Gazebo depot. You can click and drag with your mouse to fly around the 3D map from novel views while the robot continues mapping autonomously!
