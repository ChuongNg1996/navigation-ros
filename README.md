# Navigation ROS 1
Navigation on ROS 1.

## Update your Gazebo to newest version
* Follow [installation](https://classic.gazebosim.org/tutorials?tut=install_ubuntu) at *Alternative installation: step-by-step* section to update your Gazebo (default version is `x.0.0`). Note that you should check which gazebo version for your ROS version in [here](https://classic.gazebosim.org/tutorials?tut=ros_wrapper_versions&cat=connect_ros). Thus, at section 3: 

* For ROS Melodic (Gazebo 9):
  ```sh
  sudo apt-get install gazebo9
  sudo apt-get install libgazebo9-dev
  ```
* For ROS Noetic (Gazebo 11):
  ```sh
  sudo apt-get install gazebo11
  sudo apt-get install libgazebo11-dev
  ```
* After that install ROS packages for interfacing with Gazebo ([source](https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros)):

  * For ROS Melodic:
  
  ```sh
  sudo apt-get install ros-melodic-gazebo-ros-pkgs ros-melodic-gazebo-ros-control
  ```
  
  * For ROS Noetic:
  ```sh
  sudo apt-get install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control
  ```
## Install Navigation Package & Robot Localization
* Navigation Package:
  ```sh
  sudo apt update
  cd ~/catkin_ws/src
  git clone https://github.com/ros-planning/navigation_msgs
  git clone https://github.com/ros-planning/navigation -b $DISTRO-devel # $DISTRO is your ROS version, e.g.: melodic-devel
  cd ..
  catkin_make
  ```
* Robot Localization:
  ```sh
  sudo apt-get install ros-$DISTRO-robot-localization # $DISTRO is your ROS version, e.g.: ros-melodic-robot-localization
  ```

## Import 2D Map to 3D Model

### Method 1:


