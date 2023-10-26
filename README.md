Indoor navigation of a single robot in Gazebo simulation



Commands: run each command on different terminals
roscore
roslaunch turtlebot3_gazebo single_turtlebot3_world.launch
rosrun navi_gazebo globalplanner.py
rosrun navi_gazebo purepursuit.py



Given: a repository "single robot gazebo navistack"
- Indoor map of the environment
- A single mobile robot (Turtlebot 3)
- LiDAR sensor
- Localization



Task: autonomously navigate the robot to a target location
- A target location is randomly selected.
- The robot has to avoid fixed obstacles (walls, cabinets, tables).
- The robot has to avoid unknown obstacles (new objects: traffic cones, boxs, balls, cylindrical objects).



Solution:
- Global path planner: A* algorithm
- Path tracking controller: Pure pursuit



Experimental environment:
- OS (in Virtualbox): Ubuntu 20.04
- Middleware and software: ROS Noetic
- Programming language: Python (version 3x)



Video:
https://youtu.be/zpZNNnEqCd0



References:
- A* algorithm: P.E. Hart, N.J. Nilsson and B. Raphael, “A formal basis for the heuristic determination of minimum cost paths,” IEEE Transactions on Systems Science and Cybernetics, vol. 4, no. 2, pp. 100–107, 1968.
- Pure pursuit controller: Jarrod M. Snider, "Automatic Steering Methods for Autonomous Automobile Path Tracking", 2009
- PythonRobotics: https://github.com/AtsushiSakai/PythonRobotics/tree/master

