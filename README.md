# Initialpose_Apriltag

## Requirement
- ubuntu 20.04
- ros-noetic
- turtlebot3 simulation [https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation]
- Apriltag_ros package [https://github.com/AprilRobotics/apriltag_ros]

## Quickstart
. xxx

## Test
### 1. start simulation
### 2. start navigation
### 3. start Apriltag
```
roslaunch initial_pose_apriltag apriltag_simulation.launch
```
### 4. get Apriltag pose relative to map frame
```
rosrun initial_pose_apriltag get_apriltag_pose.py
```
```
rostopic pub /get_tf_map_to_apriltag std_msgs/Bool "data: true" --once
```

### 5. run initial pose service
```
rosrun initial_pose_apriltag initial_pose_service.py
```
### 6. run initial pose client
```
rosrun initial_pose_apriltag initial_pose_client.py
```

