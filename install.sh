#!/bin/bash

# Check Ubuntu version
ver=0
if [[ $(lsb_release -a | grep "Description:" | grep "Ubuntu 18") ]] ; then
	ver=18
elif [[ $(lsb_release -a | grep "Description:" | grep "Ubuntu 16") ]] ; then
	ver=16
else
	echo "This version of Ubuntu is not supported"
	exit 0
fi

# curl
sudo apt-get update
sudo apt-get -y install curl

# apt dependencies
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -
sudo apt update
sudo apt install -y python-rosinstall python-rosinstall-generator python-wstool build-essential xdotool

# ROS setup
if [ $ver -eq 18 ] ; then
	sudo apt install -y ros-melodic-desktop-full
elif [ $ver -eq 16 ] ; then
	sudo apt install -y ros-kinetic-desktop-full
fi
sudo rosdep init
sudo rosdep fix-permissions
rosdep update
if [ $ver -eq 18 ] ; then
	source /opt/ros/melodic/setup.bash
	echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
elif [ $ver -eq 16 ] ; then
	source /opt/ros/kinetic/setup.bash
	echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyACM0

# apt-get dependencies
curl -sL https://deb.nodesource.com/setup_10.x | sudo -E bash -
sudo apt-get update
sudo apt-get install -y git-core python-argparse python-vcstools python-rosdep nodejs cmake
if [ $ver -eq 18 ] ; then
	sudo apt-get install -y ros-melodic-control-msgs ros-melodic-joystick-drivers ros-melodic-xacro ros-melodic-tf2-ros ros-melodic-rviz ros-melodic-cv-bridge ros-melodic-actionlib ros-melodic-actionlib-msgs ros-melodic-dynamic-reconfigure ros-melodic-trajectory-msgs ros-melodic-rospy-message-converter ros-melodic-rosbridge-suite
elif [ $ver -eq 16 ] ; then
	sudo apt-get install -y ros-kinetic-control-msgs ros-kinetic-joystick-drivers ros-kinetic-xacro ros-kinetic-tf2-ros ros-kinetic-rviz ros-kinetic-cv-bridge ros-kinetic-actionlib ros-kinetic-actionlib-msgs ros-kinetic-dynamic-reconfigure ros-kinetic-trajectory-msgs ros-kinetic-rospy-message-converter ros-kinetic-rosbridge-suite
fi

# pip dependencies
curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py
python get-pip.py --user
rm get-pip.py
export PATH=$PATH:/home/$(uname -n)/.local/bin
pip install --upgrade --user gTTS gTTS-token pyttsx3 pygame apriltag pexpect

# Setup Intera Resources and NPM Dependencies
./setup.sh

cd robot
echo "Installation complete. Don't forget to add Firefox certificates per README.md."
