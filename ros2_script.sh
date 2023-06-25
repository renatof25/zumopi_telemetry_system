#!/bin/bash
echo ----INSTALL Python----
sudo apt install python3-pip
sudo apt install python-is-python3
sudo pip install smbus
sudo pip install RPi.GPIO
sudo pip install pyserial

echo ----ROS2----
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
#sourcing ROS2
echo source /opt/ros/humble/setup.bash >> ~/.bashrc
sudo apt install python3-colcon-common-extensions
echo source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash >> ~/.bashrc
sudo pip3 install setuptools==58.2.0
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
echo source ~/ros2_ws/install/setup.bash >> ~/.bashrc
source  ~/.bashrc

# BASH ALIASES
echo alias comprosall=\"cd ~/ros2_ws/ && colcon build\" >> ~/.bash_aliases
echo alias tel=\"ros2 launch telemetry_bringup telemetry.launch.py\" >> ~/.bash_aliases
echo alias gui=\"ros2 launch telemetry_bringup gui.launch.py\" >> ~/.bash_aliases
echo alias data=\"ros2 launch telemetry_bringup data_gen.launch.py\"  >> ~/.bash_aliases
echo alias rmlog=\"rm ~/ros2_ws/telemetry_logs/ -rf\" >> ~/.bash_aliases
source ~/.bash_aliases