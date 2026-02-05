# Robot Navigation System

A comprehensive Python project for autonomous robot navigation with goal-reaching capabilities.

## Features

- **Robot Actions**: Move forward, rotate left (30°), rotate right (30°)
- **Field of View**: 90-degree FOV for environmental awareness
- **Pathfinding**: A* algorithm for optimal path planning
- **Obstacle Avoidance**: Grid-based collision detection
- **Visualization**: Real-time visualization with matplotlib

## Project Structure

```
robot_navigation/
├── robot_navigation.py    # Main navigation system
├── demo_scenarios.py      # Multiple test scenarios
├── requirements.txt       # Dependencies
└── README.md             # Documentation
```

## Components

### 1. Robot Class
Represents the robot with:
- Position (x, y)
- Orientation (angle in degrees)
- 90° field of view
- Movement capabilities:
  - `move_forward()`: Move 1 unit forward
  - `rotate_left()`: Rotate 30° counter-clockwise
  - `rotate_right()`: Rotate 30° clockwise

### 2. GridWorld Class
Environment representation with:
- Configurable grid size
- Obstacle management
- Collision detection
- Path validation

### 3. A* Pathfinding
Optimal path planning using:
- Euclidean distance heuristic
- 8-directional movement
- Efficient priority queue implementation

### 4. RobotNavigator
High-level control system:
- Path planning integration
- Angle calculation and rotation control
- Step-by-step navigation execution
- Action sequence generation

### 5. Visualizer
Real-time visualization showing:
- Grid world and obstacles
- Robot position and orientation
- Field of view wedge
- Planned path
- Goal position

## Installation

```bash
# Install dependencies
pip install -r requirements.txt

# Or install manually
pip install numpy matplotlib
```

## Usage

### Basic Usage

```python
from robot_navigation import Robot, GridWorld, RobotNavigator, Visualizer

# Create world
world = GridWorld(width=20, height=20)
world.add_obstacle(5, 5, 3, 3)  # Add obstacles

# Create robot
robot = Robot(x=2.0, y=2.0, angle=45.0)

# Create navigator
navigator = RobotNavigator(robot, world)

# Plan and execute
if navigator.plan_to_goal(17, 17):
    actions = navigator.navigate_to_goal()
    print(f"Completed in {len(actions)} actions")
```

### Run Main Demo

```bash
python robot_navigation.py
```

This will:
1. Create a world with obstacles
2. Plan optimal path using A*
3. Execute navigation
4. Generate visualizations
5. Print action sequence

### Run Multiple Scenarios

```bash
python demo_scenarios.py
```

Includes:
- Scenario 1: Simple open space navigation
- Scenario 2: Complex maze solving
- Scenario 3: Narrow passage navigation
- Scenario 4: Multi-room environment

## Robot Specifications

- **Movement Step**: 1.0 unit per forward action
- **Rotation Step**: 30 degrees per rotation
- **Field of View**: 90 degrees
- **Rotation Directions**: Left (counter-clockwise), Right (clockwise)

## Algorithm Details

### A* Pathfinding
- **Heuristic**: Euclidean distance to goal
- **Movement**: 8-directional (including diagonals)
- **Cost Function**: f(n) = g(n) + h(n)
  - g(n): Path cost from start
  - h(n): Estimated cost to goal

### Navigation Control
1. **Path Planning**: Find optimal waypoint sequence
2. **Orientation**: Rotate to face next waypoint
3. **Movement**: Move forward toward waypoint
4. **Iteration**: Repeat until goal reached

## Output Files

The system generates visualization files:
- `robot_navigation_initial.png`: Initial state with planned path
- `robot_navigation_final.png`: Final state at goal
- `scenario_X_initial.png`: Scenario initial states
- `scenario_X_final.png`: Scenario final states

## Action Types

- **MOVE_FORWARD**: Move 1 unit in current direction
- **ROTATE_LEFT**: Rotate 30° counter-clockwise
- **ROTATE_RIGHT**: Rotate 30° clockwise

## Example Output

```
============================================================
ROBOT NAVIGATION SYSTEM
============================================================

Robot initialized at (2.0, 2.0)
Robot facing: 45.0° (FOV: 90.0°)
Goal position: (17, 17)

Planning path...
Path found with 20 waypoints

Executing navigation...

Navigation complete!
Total actions: 87
Final position: (17.12, 16.89)
Final angle: 45.00°

Action breakdown:
  MOVE_FORWARD: 71
  ROTATE_LEFT: 12
  ROTATE_RIGHT: 4
```

## Customization

### Adjust Robot Parameters

```python
robot = Robot(
    x=2.0,
    y=2.0,
    angle=45.0,
    fov=90.0,           # Field of view
    rotation_step=30.0,  # Rotation amount
    move_step=1.0        # Forward step size
)
```

### Create Custom World

```python
world = GridWorld(width=30, height=30)
world.add_obstacle(x=5, y=5, width=3, height=3)
world.add_obstacle(x=10, y=10, width=2, height=5)
```

## Key Concepts

### Field of View (FOV)
- 90° cone from robot's current orientation
- Used for environmental awareness
- Visualized as yellow wedge in graphics

### Coordinate System
- Origin (0,0) at bottom-left
- X-axis: left to right
- Y-axis: bottom to top
- Angles: 0° = right, 90° = up, 180° = left, 270° = down

### Rotation Strategy
- Always choose shortest rotation path
- 30° increments for precise control
- Adaptive direction selection (left vs right)

## Performance

- **Pathfinding**: O(n log n) where n is grid cells
- **Navigation**: Linear in path length
- **Memory**: O(n) for grid storage

## Future Enhancements

Possible extensions:
- Dynamic obstacle avoidance
- Real-time replanning
- Sensor simulation
- Multi-robot coordination
- ROS integration
- 3D navigation

## License

MIT License - Feel free to use and modify!

## Author

Created as a comprehensive robotics navigation demonstration project.
