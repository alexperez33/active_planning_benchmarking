# active_planner_benchmarking

This repository is Alex Perez's workspace for his thesis in collaboration with https://github.com/smartsystemslab-uf.
In this repository, I compare two main active path planning algorithms on a 7 dof manipulator.
The two algorithms are T-PRM and ST-RRT*.

To test out the code, do the following:
1. Clone repository in the source directory of a catkin workspace. (Ex. ~/catkin_ws/src)
2. Ensure ROS noetic is installed and sourced (source /opt/ros/noetic/setup.bash)
	 - Note, you will need to install ros-noetic-moveit-core and perhaps some other moveit libraries.
3. Enter ~/catkin_ws and compile (catkin_make)
4. There may be some ROS libraries that you need to install. Ensure to install these libraries as needed.
5. Source the ROS workspace (source ~/catkin_ws/devel/setup.bash)
6. roslaunch active_planning_pkg benchmarking.launch
7. In a separate sourced terminal, rosservice call /plan_panda "{}". This will plan the path to the goal which was defined in the launched node.
8. In the same terminal, rosservice call /execute_panda "{}". This will execute the solution which can be seen in Rviz.

To test out the dual arm kinova simulation:
1. roslaunch active_planning_pkg kinova_dual.launch
2. In a separate sourced terminal, rosservice call /plan_kinova "{}". This will plan the path to the goal which was defined in the launched node.
3. In the same terminal, rosservice call /execute_kinova "{}". This will execute the solution which can be seen in Rviz.

Resources that helped build this repository:
1. https://github.com/VIS4ROB-lab/t_prm
2. https://github.com/Kinovarobotics/ros_kortex
3. https://github.com/moveit/panda_moveit_config

