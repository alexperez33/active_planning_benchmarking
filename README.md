# active_planner_benchmarking

This repository is Alex Perez's workspace for his thesis in collaboration with https://github.com/smartsystemslab-uf.
In this repository, I compare two main active path planning algorithms on a 7 dof manipulator.
The two algorithms are T-PRM and ST-RRT*.

To test out the code, do the following:
1. Clone repository in the source directory of a catkin workspace. (Ex. ~/catkin_ws/src)
2. Ensure ROS noetic is installed along with all the necessary dependencies by running the install_script.sh in the scripts directory. This will also compile for you and throw the proper source commands in your ~/.bashrc. If you have any terminals open after running this script, restart them.
3. There are three different launch files.
	- roslaunch active_planning_pkg panda.launch
	- roslaunch active_planning_pkg kinova.launch
	- roslaunch active_planning_pkg kinova_dual.launch
4. In a separate terminal, (assuming you launched the panda) rosservice call /plan_panda "{}". This will plan the path to the goal which was defined in the launched node.
	- If you launched one of the kinova launch files you would be able to call /plan_kinova. To be sure, you can see the list of rosservices with "rosservice list".
5. In the same terminal, rosservice call /execute_panda "{}". This will execute the solution which can be seen in Rviz.

Resources that helped build this repository:
1. https://github.com/VIS4ROB-lab/t_prm
2. https://github.com/Kinovarobotics/ros_kortex
3. https://github.com/moveit/panda_moveit_config

