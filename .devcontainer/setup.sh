#!/usr/bin/env bash
# Runs once when the Codespace is created. Also works on a fresh Ubuntu 24.04
# with ROS 2 Jazzy installed (run it from the repo root with sudo available).
set -euo pipefail

REPO_DIR="$(pwd)"
SUDO=""
if [ "$(id -u)" -ne 0 ]; then SUDO="sudo"; fi

PKGS="ros-jazzy-clearpath-simulator ros-jazzy-clearpath-nav2-demos \
ros-jazzy-clearpath-desktop ros-jazzy-clearpath-config-live \
ros-jazzy-apriltag-ros ros-jazzy-ros2controlcli liburdfdom-tools \
ros-jazzy-gz-ros2-control ros-jazzy-joint-trajectory-controller \
ros-jazzy-mecanum-drive-controller ros-jazzy-rqt-image-view \
python3-colcon-common-extensions python3-pytest"

echo "==> Installing packages"
$SUDO apt-get update
$SUDO apt-get install -y wget gnupg lsb-release git

if ! $SUDO apt-get install -y $PKGS; then
  echo "==> Adding Clearpath's package server and retrying"
  wget -qO - https://packages.clearpathrobotics.com/public.key \
    | $SUDO gpg --dearmor --yes -o /usr/share/keyrings/clearpath.gpg
  echo "deb [signed-by=/usr/share/keyrings/clearpath.gpg] https://packages.clearpathrobotics.com/stable/ubuntu $(lsb_release -cs) main" \
    | $SUDO tee /etc/apt/sources.list.d/clearpath-latest.list > /dev/null
  $SUDO apt-get update
  $SUDO apt-get install -y $PKGS
fi

echo "==> Getting the KUKA iiwa 7 R800 model (lbr-stack, not on apt)"
mkdir -p "$HOME/sink_ws/src"
IIWA_DIR="$HOME/sink_ws/src/lbr_iiwa7_r800_description"
if [ ! -d "$IIWA_DIR" ]; then
  git clone --depth 1 -b v2.5.0 https://github.com/lbr-stack/lbr_iiwa7_r800_description.git "$IIWA_DIR"
fi

echo "==> Building the workspace"
ln -sfn "$REPO_DIR/sink_sim" "$HOME/sink_ws/src/sink_sim"
set +u
source /opt/ros/jazzy/setup.bash
set -u
(cd "$HOME/sink_ws" && colcon build --symlink-install)

echo "==> Robot config: Ridgeback + iiwa, front camera only (lighter without a GPU)"
mkdir -p "$HOME/clearpath"
if [ -f "$HOME/clearpath/robot.yaml" ] && \
   ! cmp -s "$HOME/clearpath/robot.yaml" "$REPO_DIR/sink_sim/clearpath/robot_front_camera_only.yaml"; then
  cp "$HOME/clearpath/robot.yaml" "$HOME/clearpath/robot.yaml.bak"
  echo "    (saved your old one as ~/clearpath/robot.yaml.bak)"
fi
cp "$REPO_DIR/sink_sim/clearpath/robot_front_camera_only.yaml" "$HOME/clearpath/robot.yaml"

if ! grep -q "sink_ws/install/setup.bash" "$HOME/.bashrc"; then
  {
    echo ""
    echo "# sink sim"
    echo "source /opt/ros/jazzy/setup.bash"
    echo "[ -f \$HOME/sink_ws/install/setup.bash ] && source \$HOME/sink_ws/install/setup.bash"
  } >> "$HOME/.bashrc"
fi

echo "==> Done. Open a new terminal and run: ros2 launch sink_sim sim.launch.py"
