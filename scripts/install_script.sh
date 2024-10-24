###### Dependencies ######
sudo apt install -y lsb-release curl gnupg

###### Install ROS ######
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

sudo apt update
sudo apt install -y ros-noetic-desktop-full

ROS_DISTRO="noetic"
ROS_SETUP="/opt/ros/$ROS_DISTRO/setup.bash"
if ! grep -q "source $ROS_SETUP" ~/.bashrc; then
  echo "Adding ROS sourcing to .bashrc..."
  echo "source $ROS_SETUP" >> ~/.bashrc
  echo "ROS setup has been added to .bashrc."
else
  echo "ROS is already sourced in .bashrc."
fi
source $ROS_SETUP

####### Install other ros dependencies ######
sudo apt install -y ros-noetic-moveit-core
sudo apt install -y ros-noetic-moveit-ros-planning
sudo apt install -y ros-noetic-moveit-ros-planning-interface
sudo apt install -y ros-noetic-moveit-ros-perception
sudo apt install -y ros-noetic-rviz-visual-tools
sudo apt install -y ros-noetic-moveit-visual-tools
sudo apt install -y ros-noetic-moveit-planners-ompl
sudo apt install -y ros-noetic-franka-description

####### Compile #######
# Locate the workspace directory relative to this script's location
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

echo "Building the workspace: $WORKSPACE_DIR"

# Build the workspace using catkin_make with the -C flag
catkin_make -C "$WORKSPACE_DIR"

# Check if the build was successful
if [ $? -eq 0 ]; then
  echo "Compilation successful."

  # Check if the workspace's setup.bash is already sourced in .bashrc
  if ! grep -q "source $WORKSPACE_DIR/devel/setup.bash" ~/.bashrc; then
    echo "Adding workspace to .bashrc..."
    echo "source $WORKSPACE_DIR/devel/setup.bash" >> ~/.bashrc
    echo "Workspace setup added to .bashrc."
  else
    echo "Workspace is already sourced in .bashrc."
  fi
else
  echo "Error: Compilation failed."
  exit 1
fi
source ~/.bashrc
