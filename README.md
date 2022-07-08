# Navigation ROS
Navigation on ROS.

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

## Practice with Turtlebot 3 Simulation.
* Follow [installation](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup) except *1.1.2 Install ROS on Remote PC* if you have ROS installed and *1.1.6 Network Configuration*, choose correct tab for your ROS version (e.g. Kinetic, Melodic, etc.).
* Go to [6.Simulation](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/), do *6.1, 6.2, 6.3*.

  At section 6.1,  

