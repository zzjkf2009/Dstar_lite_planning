# Dynamic planning using Dstar_lite

## Overview

This is the project about using Dynamic planning algorithm D* lite to calculate the
path for Turtlebot from start position to the goal position in a dynamic unknown environment.
The original map is known - the static obstacle is known before head by mapping using the *slam_gmapping* (one package from the navigation stack).It provides a laser-based SLAM and can generate a 2D occupancy map by given laser scan data and pose data (Odometry).More details will be introduced later. After we got the occupancy map, D* lite algorithm (as a plugin in *move_base* global planner) will be implemented to generate a path from current to goal position. The planned path will be regenerated (smoothed )to a spline using minimal acceleration trajectory method.Meanwhile,*amcl* will be used to localize the robot by applying a particle filter to track its pose against the known map. And a PID controller is designed to control the robot to move along desired path (*move_base*). However, the environment will be dynamic and robot has no idea about those dynamic changes before head. As the robot moves, it will encounter the situation that the path is blocked or there is a shortcut, in that case, the robot will detect those changes with scanner (LIDAR or laser-scanner). Then the map will be updated accordingly and a new path will be generated with D* lite. And the robot will respond to the new path until it reach the goal.

## Todo
One of the drawback for D*lite is that, when the robot observe a change of the environment, it will regenerate a new path, this process takes time (depends on the environment, maybe a few milliseconds up to few seconds). And the question is that, what does the robot do during this time. Maybe you want the robot stops at there until it finds the new path or slow down. But what if the robot has high momenta or speed or it can't maintain a stop state (like airplane). So *anytime algorithm* need to be applied in future work.
Also, this could be challenge to apply it on a drone. There is a package *hector_quadrotor* which provide a platform to simulate the behavior of the drone. I will try it (maybe Anytime dynamic A*) on it. The quad are equipped with a modern LIDAR systems like the Hokuyo UTM-30LX. Package *hector_mapping* will in turn generate the occupancy map just like *slam_gmapping*. But instead of using Odometry, it uses data from LIDAR and IMU.

## Run
```
roslaunch Dstar_lite_planning turtlebot_garage.launch
roslaunch Dstar_lite_planning turtlebot_dstar_navigation.launch
```
1. Change global planner to our own global planner

2. To send a goal:

    - Click the "2D Nav Goal" button

    - Click on the map where you want the TurtleBot to drive and drag in the direction the TurtleBot should be pointing at the end.

## Pipeline
- Map_building: As said in the overview, the package *slam_gmapping* is used to generate the 2D occupancy map. Originally, it uses the tf from /Odometry as pose of the robot. This /tf can be inaccurate due to uneven terrains or drift and need to be optimized with other data, such as gyro (IMU). So an sensor fusion package *robot_pose_ekf* is used to estimate a optimal pose by combining the odometer and gyro using extended kalman filter. Thus, the /tf from robot_pose_ekf/odom_combined (topic) will be used instead to feed into the *slam_gmapping*, which will gives us a occupancy map.
- Localization: Localization is done by package *amcl* which takes in a laser-baser map, laser scans, and transforms messages, and return pose estimates. It implements the adaptive Monte Carlo localization, which uses particle filter to track the pose of the robot against a known map.
- D* lite is a dynamic re-planning algorithm based on LPA* (lifelong planning A*), which can do the re-planning based on the updated map. To get a deep understand of the algorithm, refer this paper:http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf

## Result
The video shows how the Turtlebot navigate through the changed (comparing to the mapped environment) terrain, and update (re-plan) its global path according to the change.

- [![Refer video](https://img.youtube.com/vi/xUx7TYDRqXA/0.jpg)](https://youtu.be/xUx7TYDRqXA)
## Update:
- Due to the drift of the Odometry, the mapping info was not accurate if the robot is operating for a long time mapping. This cause problem since the localization is depend on the map and if the localization is massed up, the observation will have a hard time to match the map, which in turn will cause the robot spend most of its time on planning (see (how D*lite works)[http://idm-lab.org/bib/abstracts/papers/aaai02b.pdf]). so *slam_gmapping* will become a secondary choice, I decide to use LIDAR (*hector_mapping*) instead, which may give a much better (accurate and precise) map.
- I decide to create the implement the algorithm on the hard code grid map first, since the accuracy of the map will influence the overall performance, to focus more on the planning algorithm itself on this stage, I will use a simple hard coded map first, and then move onto scanned map for nest stage.
## Resources
SLAM package:
- For kinect: *RGBD-6D-SLAM*.  After convert it to laser data using face laser scanner, *slam_gmapping* can be used
- For LIDAR: *hector_mapping*
- Mapping and localization: [Upenn lecture](https://www.youtube.com/watch?v=Q4qM-Uzj1SI)
- [Writing A Global Path Planner As Plugin in ROS](http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS)

Navigation Plugin in ROS:
- Globalplanner:
http://wiki.ros.org/navigation/Tutorials/Writing%20A%20Global%20Path%20Planner%20As%20Plugin%20in%20ROS
## Planning algorithms
- LPA* (lifelong planning A*)
- D* lite
## Knowledge
This global planner plug-in was inspired by Palmieri's (palmieri@informatik.uni-freiburg.de) srl_dstar_lite (https://github.com/palmieri/srl_dstar_lite), modification and improvements are made accordingly.
