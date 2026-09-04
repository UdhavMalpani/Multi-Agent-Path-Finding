#!/usr/bin/env bash
# install_ros.sh — Installs ROS2 Humble + Gazebo-ROS2 bridge on Ubuntu 22.04
# Run: chmod +x install_ros.sh && ./install_ros.sh

set -euo pipefail
GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info() { echo -e "${CYAN}[MAPF]${NC} $1"; }
ok()   { echo -e "${GREEN}[ OK ]${NC} $1"; }

info "Starting ROS2 Humble + Gazebo installation for Ubuntu 22.04 (Jammy)..."

# ── 1. Locale ──────────────────────────────────────────────────────────────
info "Configuring locale..."
sudo apt-get update -qq
sudo apt-get install -y -qq locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
ok "Locale set"

# ── 2. Universe repo ───────────────────────────────────────────────────────
info "Enabling Ubuntu Universe repository..."
sudo apt-get install -y -qq software-properties-common
sudo add-apt-repository -y universe
ok "Universe enabled"

# ── 3. ROS2 apt key + repo ─────────────────────────────────────────────────
info "Adding ROS2 apt repository..."
sudo apt-get install -y -qq curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
  http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt-get update -qq
ok "ROS2 repository added"

# ── 4. Install ROS2 Humble + Gazebo bridge ─────────────────────────────────
info "Installing ROS2 Humble Desktop + Gazebo ROS packages (this may take ~5 min)..."
sudo apt-get install -y \
  ros-humble-desktop \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-plugins \
  ros-humble-gazebo-ros \
  ros-humble-xacro \
  ros-humble-nav-msgs \
  ros-humble-geometry-msgs \
  ros-humble-visualization-msgs \
  ros-humble-tf2-ros \
  ros-humble-tf2-geometry-msgs \
  ros-humble-robot-state-publisher \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-pip \
  python3-numpy
ok "ROS2 Humble installed"

# ── 5. rosdep init ─────────────────────────────────────────────────────────
info "Initialising rosdep..."
sudo rosdep init 2>/dev/null || info "rosdep already initialised"
rosdep update
ok "rosdep ready"

# ── 6. Shell setup ─────────────────────────────────────────────────────────
BASHRC="$HOME/.bashrc"
SETUP_LINE="source /opt/ros/humble/setup.bash"
if ! grep -qF "$SETUP_LINE" "$BASHRC"; then
  echo "$SETUP_LINE" >> "$BASHRC"
  ok "Added ROS2 source to ~/.bashrc"
fi

# ── 7. Verify ──────────────────────────────────────────────────────────────
source /opt/ros/humble/setup.bash
info "Checking ROS2..."
ros2 --version
ok "ROS2 Humble is ready!"

echo ""
echo -e "${GREEN}════════════════════════════════════════════════${NC}"
echo -e "${GREEN}  Installation complete!${NC}"
echo -e "${GREEN}  Next step: ./build.sh${NC}"
echo -e "${GREEN}════════════════════════════════════════════════${NC}"
