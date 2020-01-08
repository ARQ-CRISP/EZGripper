# EZGripper

This is our fork of https://github.com/SAKErobotics/EZGripper, containing everything required to run the Gripper with our framework!

## Install the EZGripper ROS Driver (Indigo or Kinetic)

1) Install the python EZGripper library https://github.com/SAKErobotics/libezgripper

2) Install dependencies:
```
sudo apt install ros-<distro_version>-joystick-drivers
```

3) Download and install the package
```
cd ~/catkin_ws/src
git clone https://github.com/ARQ-CRISP/EZGripper.git
cd ..
catkin_make
```
