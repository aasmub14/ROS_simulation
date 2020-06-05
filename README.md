# ROS_simulation
To be developed under water manipulator simulator
					How to run the simulation
-------------------------------------------------------------------------------------------------------
Setup:
	The simulation runs on the following software: Ubuntu 18.0.4, ROS Melodic and Gazebo 9.0.
The mentioned programs can be downloaded and installed by following the instructions in these urls:

-Ubuntu 18.0.4	https://www.linuxtechi.com/ubuntu-18-04-lts-desktop-installation-guide-screenshots/
-ROS Melodic	http://wiki.ros.org/melodic/Installation/Ubuntu
-Gazebo 9.0	http://gazebosim.org/tutorials?tut=install_from_source&cat=install

Alternatively could a virtual machine be setup to run Ubuntu on another OS, but the simulation is 
not tested for this case.
 
Note: Before starting this simulation should the user either have some experience with ROS and Gazebo 
or atleast have a look at the beginner tutorial for ROS(http://wiki.ros.org/ROS/Tutorials) and then
 move on to more advanced tutorials including Gazebo(http://gazebosim.org/tutorials)

First create a catkin workspace(See ROS beginner tutorials) and source the ROS environment using the 
command in a terminal(CTRL+ALT+t):

	$ source /devel/setup.bash

This should ensure that ROS commands can be used. If everything got installed successfully, copy 
the two packages from this repository:

	https://github.com/aasmub14/ROS_simulation.git

into the /src/ folder in the catkin workspace. Then navigate to the /src/ folder in the terminal 
and run the command:

	$ rosdep install PACKAGE_NAME

for each of the packages found in the repository. This will then find and install the necessary 
dependencies, other packages, needed to run the simulation.
-------------------------------------------------------------------------------------------------------
Start up:
	Open a terminal and find the location of /ros_underwater_simulator(roscd) and 
run the command:

	$ roslaunch ros_underwater_simulator controllable.launch model:=urdf/uw_arm_control_ee.urdf

A Gazebo window should pop up with the model of the robot.

Next open another terminal and find the same package, but change directory to /nodes/. Then 
run the command:

	$ python teleop_final.py

to launch the command node. This node takes the inputs from keybindings; '1','2','Q' and 'W' 
to control the two driven joints.

Then open a third terminal and find the package /uw_arm_control/. This package contains the 
controller instructions and is started by running the command:

	$ roslaunch uw_arm_control controller.launch

Now should the manipulator be controllable by the aforementioned key inputs.
-------------------------------------------------------------------------------------------------------
Running:
	In order to monitor the controllers and the joints can a terminal be opened and enter the 
command:

	$ rqt

This opens a GUI where ROS topics can be plotted by selecting 'plugins->visualization->plot' and 
then select the topic of interest. 
The same can be done in Gazebo by selecting 'window->plot' and select the Gazebo topic of interest.
-------------------------------------------------------------------------------------------------------
Disclaimer:
	We are by no means experts with this framework and the model is far from representing the 
idea of an underwater manipulator(No water physics would tip this off), but feel free to send questions
 and improve on it.

