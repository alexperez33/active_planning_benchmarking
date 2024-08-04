# active_planner_benchmarking

This repository is Alex Perez's workspace for his thesis in collaboration with https://github.com/smartsystemslab-uf.
In this repository, I compare two main active path planning algorithms on a 7 dof manipulator.
The two algorithms are T-PRM and ST-RRT*.

To test out the code, do the following:
1. Clone repository in the source directory of a catkin workspace. (Ex. ~/catkin_ws/src)
2. Inside the active_planning_benchmarking directory, run the command to update the submodules (git submodule update --init --recursive)
3. Ensure ROS noetic is installed and sourced (source /opt/ros/noetic/setup.bash)
4. Enter ~/catkin_ws and compile (catkin_make)
5. There may be some ROS libraries that you need to install. Ensure to install these libraries as needed.
6. Source the ROS workspace (source ~/catkin_ws/devel/setup.bash)
7. roslaunch panda_alex_pkg benchmarking.launch
8. In a separate sourced terminal, rosservice call /plan_panda "{}". This will plan the path to the goal which was defined in the launched node.
9. In the same terminal, rosservice call /execute_panda "{}". This will execute the solution which can be seen in Rviz.

To test out the dual arm kinova simulation:
1. roslaunch panda_alex_pkg kinova_dual.launch
2. In a separate sourced terminal, rosservice call /plan_kinova "{}". This will plan the path to the goal which was defined in the launched node.
3. In the same terminal, rosservice call /execute_kinova "{}". This will execute the solution which can be seen in Rviz.
