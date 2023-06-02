#!/bin/bash
echo ----UPDATE----
sudo apt update
sudo apt upgrade
sudo reboot

echo ----Various Tools Installations----
sudo apt install net-tools
sudo apt install openssh-server
sudo apt install xrdp

echo ----INSTALL USB WIFI DONGLE DRIVER----
sudo apt install -y build-essential dkms git
sudo apt install -y build-essential dkms git iw
sudo apt install linux-headers-$(uname -r)
mkdir -p ~/src
cd ~/src
git clone https://github.com/morrownr/88x2bu-20210702.git 
cd ~/src/88x2bu-20210702
sudo ./install-driver.sh
sudo apt install ifstat

echo ----INSTALL ARDUINO SOFTWARE----
sudo apt install arduino
sudo apt install curl
# arduino-cli
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh
echo 'export PATH=$PATH:/home/pi/bin' >> ~/.bashrc
source ~/.bashrc
#arduino cores
arduino-cli core install arduino:avr

echo ----INSTALL Python----
sudo apt install python3-pip
sudo apt install python-is-python3
sudo pip install smbus
sudo pip install RPi.GPIO
sudo pip install pyserial

echo ----UPS SCRIPTS----
cd ~
git clone https://github.com/geekworm-com/x728
sudo bash ~/x728/x728-v2.1.sh

echo ----camera----
sudo apt-get -y install v4l-utils
sudo apt install ffmpeg

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
echo \# arduino cli >> ~/.bash_aliases
echo alias compileZumo=\"arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4\" >> ~/.bash_aliases
echo alias uploadZumo=\"arduino-cli upload -p /dev/ttyACM0 --fqbn pololu-a-star:avr:a-star32U4\" >> ~/.bash_aliases
echo \# Telemetry aliases >> ~/.bash_aliases
echo alias zardrun=\"compileZumo ~/arduino_serial_interface && uploadZumo ~/arduino_serial_interface\" >> ~/.bash_aliases
echo alias comprosall=\"cd ~/ros2_ws/ && colcon build\" >> ~/.bash_aliases
echo alias tel=\"ros2 launch telemetry_bringup telemetry.launch.py\" >> ~/.bash_aliases
echo alias gui=\"ros2 launch telemetry_bringup gui.launch.py\" >> ~/.bash_aliases
echo alias rmlog=\"rm ~/ros2_ws/telemetry_logs/ -rf\" >> ~/.bash_aliases
