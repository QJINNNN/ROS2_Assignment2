# ROS2_Assignment2

## How to run
### 1. Spawn Obstacle(turtle1)
```python
ros2 run turtlesim turtlesim_node
```

### 2. Spawn Robot(turtle2)
```python
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1, y: 1, theta: 0, name: 'turtle2'}"
```

### 3. Receiving turtle2's present location
```python
ros2 topic echo /turtle2/pose
```

### 4. Build
```python
cd ~/your_directory
colcon build --packages-select planner
. ~/your_directory/install/local_setup.bash
```

### 5. Run
```python
ros2 run planner dwa_planner
```
