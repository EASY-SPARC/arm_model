# arm_model  
## About the Project  
This simulational system represents a cylindrical **mobile base** with a **robotic manipulator**.  
### The Mobile Base  
The mobile base moves as a prismatic joint over x-axis attached to a continuous joint rotating in z-axis. This implementation tries to resamble kobuki's movement.  
### The Manipulator
Placed over the moving base, this manipulator has 4 degrees of liberty, one o which is a redundant rotation of the basis, and a simbolic fixed effectuator.  

## Dependencies
* [ROS Melodic](http://wiki.ros.org/melodic)  
* [Gazebo*](http://gazebosim.org/)  
* [Rviz*](http://wiki.ros.org/rviz)  
* [MoveIt](https://moveit.ros.org/)  
* [Python 2.7](https://www.python.org/download/releases/2.7/)  
*Usually acquired in a complete ROS installation

## Libraries Installation
### ROS
Follow instructions in [ROS Installation](http://wiki.ros.org/melodic/Installation) to install the complete version of the official repository as used in this project. Or simply run the folowing:  
`$ sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'`  
`$ sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654`  
`$ sudo apt update`  
`$ sudo apt install ros-melodic-desktop-full`  
### ROS Packages Needed
To run this simulation you must have installed the following packages:  
`$ sudo apt install ros-melodic-effort-controllers`  
`$ sudo apt install ros-melodic-velocity-controllers`  
`$ sudo apt install ros-melodic-joint-state-*`  
`$ sudo apt install ros-melodic-joint-trajectory-controller`  
### MoveIt  
To acquire MoveIt, just run as below:  
`$ sudo apt install ros-melodic-moveit`  

## Running the Simulation
Run:
`roslaunch manipulator_moveit_config setup_position.launch`  
