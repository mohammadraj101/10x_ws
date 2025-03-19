## 10x_ws

This repository consists of assignments given to make DWA Local planner without using 

### Objective

Create a ROS2 package with a node for video conversion in ROS2.

### Video Submission

https://drive.google.com/drive/folders/1DjR_KlYWfcxx8S4E6fx437Lm58LPWZeV?usp=sharing

### Quick Setup

1. Clone the repository
```sh
git clone https://github.com/mohammadraj101/mowito_ws.git src/
```
2. Set up turtlebot3
```sh
sudo apt install -y ros-humble-turtlebot3* 
export TURTLEBOT3_MODEL=waffle'
 ```
3. Install dependencies
```sh
rosdep install --from-paths src --ignore-src -r -y
```
4. Build the workspace

Make sure you have sourced the ROS2 environment before building:

```sh
source /opt/ros/humble/setup.bash
colcon build 
```
5. Source the workspace

After a successful build, source the workspace:
```sh
source install/setup.bash
```
6. Run the launch file

This will launch the gazebo and rviz:
```sh
ros2 launch turtlebot3_gazebo turtlebot3_my_world.launch.py
```
7.Run the Service node containing the DWA Planner.

```sh
ros2 run dwa_planner dwa_planner_node 
```
8. Test the goal by calling the service

To test the service call:

x, y are the goal location.
```sh
ros2 service call /get_goal dwa_planner/srv/GetGoal "{x: 5.0, y: 0.0}"
```

#### Package Structure
```sh
src
├── dwa_planner
│   ├── CMakeLists.txt
│   ├── include
│   │   └── custom_dwa_planner
│   ├── package.xml
│   ├── scripts
│   ├── src
│   │   ├── dwa_planner_node.cpp
│   │   └── working_backup_my_world.cpp
│   └── srv
│       └── GetGoal.srv
├── README.md
```

Contributors

RAJ MOHAMMAD



