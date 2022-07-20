# Navigation ROS 1
Navigation on ROS 1.

## Update your Gazebo to newest version
* Possible dependencies:
  ```sh
  sudo apt-get update
  sudo apt upgrade libignition-math2
  ```
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
  
  If installation by `apt` is failed, install ROS-Gazevo interface from source:
  ```sh
  cd ~/catkin_ws/src
  git clone https://github.com/ros-simulation/gazebo_ros_pkgs.git -b $DISTRO-devel # $DISTRO is your ROS version, e.g.: melodic-devel
  rosdep update
  rosdep install --from-paths . --ignore-src --rosdistro=$DISTRO -y # $DISTRO is your ROS version, e.g.: melodic-devel
  cd ~/catkin_ws/
  catkin_make
  ```
  
## Installations
* Navigation Package:
  ```sh
  sudo apt update
  cd ~/catkin_ws/src
  git clone https://github.com/ros-planning/navigation_msgs
  git clone https://github.com/ros-planning/navigation -b $DISTRO-devel # $DISTRO is your ROS version, e.g.: melodic-devel
  rosdep update
  rosdep install --from-paths . --ignore-src --rosdistro=$DISTRO -y # $DISTRO is your ROS version, e.g.: melodic-devel
  cd ..
  catkin_make
  ```
* Robot Localization:
  ```sh
  sudo apt-get install ros-$DISTRO-robot-localization # $DISTRO is your ROS version, e.g.: ros-melodic-robot-localization
  ```
  
## Example 1: Use Navigation & Robot Localization Packages
* Unlock the model folder of Gazebo so you can import model:
  ```sh
  sudo chmod a+rwx /usr/share/gazebo-9/models
  ```
  Then copy & paste `sample_1` folder to `/usr/share/gazebo-9/models`
* Run the 1st example"
  ```sh
  roslaunch diff_drive_example diff_drive_example_community.launch 
  ```
  On Rviz, click on `2D Nav Goal`, drag and drop on nearby zone of the robot and observe the autonomous navigation.

## Example 2: Use a set of waypoints
* After *Example 1*, close all and open new terminals:

  On Terminal 1:
  ```sh
  roslaunch diff_drive_example diff_drive_example_community.launch 
  ```
  On Terminal 2:
  ```sh
  roslaunch diff_drive_example diff_drive_example_community_waypoints.launch 
  ```
  Observe that robot moves to set waypoints autonomously. The waypoints can be seen and modified in `set_goals_1.cpp`
  
## Import 2D Map to 3D Model

### Method 1:
* Install [OpenCV](https://docs.opencv.org/4.x/d7/d9f/tutorial_linux_install.html).
* Install dependencies of `map2gazebo`:
  ```sh
  pip install --user trimesh
  pip install --user numpy
  pip install --user pycollada
  pip install --user scipy
  pip install --user networkx
  ```
* Install both ROS Packages `load_map` and `map2gazebo`.
* Change your image to greyscale & threshold with `rgb_to_grey.cpp` (or any equivalent tool). Put image in same folder `thresholding_grey`, adjust line 12 `Mat img_grayscale = imread("$image", IMREAD_GRAYSCALE);` where `$image` is your image ( e.g. `Mat img_grayscale = imread("100_100_room.png", IMREAD_GRAYSCALE);`), then open folder in terminal:
  ```sh
  cmake .
  make
  ./rgb_to_grey
  ```
* Use [GIMP](https://www.gimp.org/) to load the generated image and export it again as m`.pgm` with `ASCII` value. 
* Open `~/catkin_ws/src/load_map/map`, study one sample. Each sample has `.pgm` which is the image and `.yaml` file which is its description. Do the same for generated image.
* Open `load_map.launch` in `~/catkin_ws/src/load_map/launch` and modify `<arg name="map_file" default ="$(find load_map)/map/$image.yaml" />` where `$image` is your `.yaml` file (e.g. `<arg name="map_file" default ="$(find load_map)/map/sample_2.yaml" />`). Then launch it to load the map to `map_server`:
  ```sh
  roslaunch load_map load_map.launch
  ```
* In `map2gazebo.py`, depends on your **OpenCV** version, line 73,74 can be:
  ```sh
  image, contours, hierarchy = cv2.findContours(
         thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE) # for OpenCV 2
  ```
  or
    ```sh
  contours, hierarchy = cv2.findContours(
         thresh_map, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE) # for OpenCV 3 or above
  ```
* while `load_map.launch` is running, run `map2gazebo.launch` to export `.stl` model:
  ```sh
  roslaunch map2gazebo map2gazebo.launch export_dir:=/path/to/export_dir # Export to a selected folder "/path/to/export_dir"
  # e.g.: roslaunch map2gazebo map2gazebo.launch export_dir:=$HOME//maps
  ```
## Questions

<p align="center">
  <img width="600" height="250" src="./images/nav_stack.png">
</p>
