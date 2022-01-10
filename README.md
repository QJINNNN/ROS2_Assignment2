# ROS_Assignment2

## How to run
### 1. Spawn Obstacle(turtle1)
```python
ros2 run turtlesim turtlesim_node
```

### 2. Spawn Robot(turtle2)
```python
ros2 service call /spawn turtlesim/srv/Spawn "{x: 1, y: 1, theta: 0.2, name: 'turtle2'}"
```

### 3.
