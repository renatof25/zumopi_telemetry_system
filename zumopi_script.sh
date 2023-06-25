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

echo ----UPS SCRIPTS----
cd ~
git clone https://github.com/geekworm-com/x728
sudo bash ~/x728/x728-v2.1.sh

echo ----camera----
sudo apt-get -y install v4l-utils
sudo apt install ffmpeg

# BASH ALIASES
echo \# arduino cli >> ~/.bash_aliases
echo alias compileZumo=\"arduino-cli compile --fqbn pololu-a-star:avr:a-star32U4\" >> ~/.bash_aliases
echo alias uploadZumo=\"arduino-cli upload -p /dev/ttyACM0 --fqbn pololu-a-star:avr:a-star32U4\" >> ~/.bash_aliases
echo \# Telemetry aliases >> ~/.bash_aliases
echo alias zardrun=\"compileZumo ~/arduino_serial_interface && uploadZumo ~/arduino_serial_interface\" >> ~/.bash_aliases
source ~/.bash_aliases