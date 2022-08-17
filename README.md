# gazebo_gripping_plugin

A plugin to mimic gripping mechanisms for Gazebo simulation and ROS.

# Overview

This plugin provides "fake" gripping effect done on links of robots when a contact is detected to the target link (supposedly your gripper). It simply creates a virtual fixed joint as long as the gripping is commanded and detaches the joint when a gripping release is requested.

  

# Installation 
	$ cd <path-to-your-workspace>/src
	$ git clone https://github.com/MichaelHannalla/gazebo_gripping_plugin.git
	$ cd ..
	$ catkin_make

# Usage 
To use this plugin, you need to add the following snippet to your robot's URDF file, you must then set the `gripper_link_name` tag to the name of the link that you wish to grip objects with. Please note that such link **MUST have a collision enabled** in your URDF file.

	<gazebo>
		<plugin  name="gazebo_gripping_plugin"  filename="libgazebo_gripping_plugin.so">
			<gripper_link_name>gripper_link</gripper_link_name>
		</plugin>
	</gazebo>
    
After the plugin has been loaded successfully, you should see a message `[ INFO]: [Gripping Plugin] plugin successfully loaded` when loading your model in g
azebo.

You'll then notice that a topic of named `/gripper_attach_cmd` with type `std_msgs/Bool`. Publish `true`/`false` to `grip`/`release` when the object you want is in contact with your gripper link you specified by `gripper_link_name` tag in your URDF file.


# Maintainer

This code has a lot of optimizations to be done so PRs are very welcomed. \
Code developed and maintained by: [Michael Samy Hannalla](https://www.linkedin.com/in/MichaelHannalla)