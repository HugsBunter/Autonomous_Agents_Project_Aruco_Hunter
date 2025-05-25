# Aruco_Hunter

The project integrates ROS, Gazebo, Python, and OpenCV to create a dynamic robotic system. It's about a 4-wheel vehicle robot with a camera sensor, which will recognize a specific Aruco id (in this case id 5) in an area and has to chase it. The robot adjusts its trajectory in real-time to follow the target, which can be either stationary or moving. The robotic autonomous agent will also have to execute a specific routine, while it doesn’t identify its specific Aruco target and scan the area for it, by ignoring any other Aruco ids. The search operation continues until the time limit is reached (2 minutes) and our hunter is stuck in eternity (or until an aruco with id 5 appears in front of it). This project showcases the potential of autonomous agents in real-time target tracking and identification scenarios.

- Dependencies

1. Python (version 3.8.10)
2. ROS (Robot Operating System) Noetic Ninjemys
3. OpenCV (version 4.2.0)
4. Gazebo (version 11.11.0)

After installing all dependencies, add the following line to your .bashrc file:

source /opt/ros/noetic/setup.bash



- Workspace Configuration

Please replace `(your_path)` with the actual path to your `catkin_ws` directory. The name of the workspace for this project is catkin_ws and you are going to create it in the last step of installing ROS. You can use your own name for the workspace, but for the purposes of this README, we will use catkin_ws. After that you should:

1. Copy or clone the package files (mobile_robot) to (your_path)/catkin_ws/src

2. Install all the Debian package dependencies listed in the mobile_robot/package.xml files with 	the following command:
	$ rosdep install

3. Navigate to (your_path)/catkin_ws and build your workspace using the following command:
	$ catkin_make

4. Source the current environment variables with the following command:
	$ source devel/setup.bash 
   You can also add this line to your .bashrc file like:
	source (your path)/catkin_ws/devel/setup.bash
	
5. Set the GAZEBO_PLUGIN_PATH enviroment variable with the following command:
	$ export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:(your path)/catkin_ws/src/mobile_robot/  		  plugins/build
   You can also add it to your .bashrc file.

6. After you edit the .bashrc file, you can either start a new shell or source the ~/.bashrc file 	 in your current shell with the following command: 
	source ~/.bashrc
	
7. Navigate to the .gazebo/models directory (usually located in the Home folder) and copy all     contents from source_code/models to .gazebo/models.    



- Running the Simulation
	
To run the simulation, open two terminals. In the first terminal, run the following command:
	$ roslaunch mobile_robot gazebo.launch

In the second terminal, run the following command:
	$ rosrun mobile_robot aruco_rec.py 
	
Once the Gazebo simulation is loaded, press the play button at the bottom of the application window.
