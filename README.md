# Initialpose_Apriltag

## Requirement
- ubuntu 20.04
- ros-noetic
- turtlebot3 simulation [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation]
- Apriltag_ros package [https://github.com/AprilRobotics/apriltag_ros]

## Quickstart
1. use waffle_pi model
```
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
```
2. create Apriltag_box model to gazebo path
```
cd your_ws/src/Initialpose_Apriltag/initial_pose_apriltag/models/ && cp -r Apriltag_box ~/.gazebo/models
```

## Test
### 1. start simulation
```
roslaunch initial_pose_apriltag spawn_turtlebot_with_apriltag.launch
```
### 2. start navigation
create map https://emanual.robotis.com/docs/en/platform/turtlebot3/slam_simulation/ before start navigation
- 2.1 launch start navigation
```
roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/map.yaml
```
- 2.2 navigate robot to move camera to apriltag location
### 3. start Apriltag
```
roslaunch initial_pose_apriltag apriltag_simulation.launch
```
### 4. get Apriltag pose relative to map frame

- 4.1 start node
```
rosrun initial_pose_apriltag get_apriltag_pose.py
```
- 4.2 start pub data
```
rostopic pub /get_tf_map_to_apriltag std_msgs/Bool "data: true" 
```
- 4.3 save data to config/apriltag_to_map_pose.yaml
```
rostopic pub /save_apriltag_pose std_msgs/Bool "data: true" --once
```
- 4.4 close get_apriltag_pose.py node

### 5. initial robot pose
- 5.1 run initial pose service
```
rosrun initial_pose_apriltag initial_pose_service.py
```
- 5.2 run initial pose client
```
rosrun initial_pose_apriltag initial_pose_client.py
```

