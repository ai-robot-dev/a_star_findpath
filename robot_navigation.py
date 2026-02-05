"""
Robot Navigation System with A* Pathfinding
Objective: Navigate a robot to a goal position with FOV constraints
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, Rectangle
from collections import deque
import heapq
from dataclasses import dataclass
from typing import List, Tuple, Optional
import math


@dataclass
class Robot:
    """Robot with position, orientation, and movement capabilities"""
    x: float
    y: float
    angle: float  # in degrees, 0 = right, 90 = up
    fov: float = 90.0  # Field of view in degrees
    rotation_step: float = 30.0  # Rotation amount in degrees
    move_step: float = 1.0  # Forward movement step
    
    def move_forward(self):
        """Move robot forward in current direction"""
        rad = math.radians(self.angle)
        self.x += self.move_step * math.cos(rad)
        self.y += self.move_step * math.sin(rad)
    
    def rotate_right(self):
        """Rotate robot 30 degrees clockwise"""
        self.angle -= self.rotation_step
        self.angle %= 360
    
    def rotate_left(self):
        """Rotate robot 30 degrees counter-clockwise"""
        self.angle += self.rotation_step
        self.angle %= 360
    
    def get_position(self) -> Tuple[float, float]:
        """Return current position"""
        return (self.x, self.y)
    
    def get_fov_bounds(self) -> Tuple[float, float]:
        """Return FOV angle bounds (start, end)"""
        half_fov = self.fov / 2
        start_angle = self.angle - half_fov
        end_angle = self.angle + half_fov
        return (start_angle, end_angle)
    
    def can_see_point(self, px: float, py: float) -> bool:
        """Check if a point is within robot's field of view"""
        dx = px - self.x
        dy = py - self.y
        
        # Calculate angle to point
        angle_to_point = math.degrees(math.atan2(dy, dx))
        
        # Get FOV bounds
        start_angle, end_angle = self.get_fov_bounds()
        
        # Normalize angles
        angle_to_point %= 360
        start_angle %= 360
        end_angle %= 360
        
        # Check if point is within FOV
        if start_angle <= end_angle:
            return start_angle <= angle_to_point <= end_angle
        else:  # FOV crosses 0 degrees
            return angle_to_point >= start_angle or angle_to_point <= end_angle


class GridWorld:
    """Grid-based world with obstacles"""
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = np.zeros((height, width), dtype=int)
        self.obstacles = []
    
    def add_obstacle(self, x: int, y: int, width: int = 1, height: int = 1):
        """Add rectangular obstacle to grid"""
        for i in range(y, min(y + height, self.height)):
            for j in range(x, min(x + width, self.width)):
                if 0 <= i < self.height and 0 <= j < self.width:
                    self.grid[i][j] = 1
        self.obstacles.append((x, y, width, height))
    
    def is_valid(self, x: int, y: int) -> bool:
        """Check if position is valid (within bounds and not obstacle)"""
        if x < 0 or x >= self.width or y < 0 or y >= self.height:
            return False
        return self.grid[int(y)][int(x)] == 0
    
    def is_path_clear(self, x1: float, y1: float, x2: float, y2: float, steps: int = 10) -> bool:
        """Check if straight path between two points is clear"""
        for i in range(steps + 1):
            t = i / steps
            x = x1 + t * (x2 - x1)
            y = y1 + t * (y2 - y1)
            if not self.is_valid(int(x), int(y)):
                return False
        return True


class AStarPlanner:
    """A* pathfinding algorithm for robot navigation"""
    
    @staticmethod
    def heuristic(pos1: Tuple[int, int], pos2: Tuple[int, int]) -> float:
        """Euclidean distance heuristic"""
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    @staticmethod
    def find_path(world: GridWorld, start: Tuple[int, int], goal: Tuple[int, int]) -> Optional[List[Tuple[int, int]]]:
        """Find optimal path using A* algorithm"""
        # Priority queue: (f_score, counter, position, path)
        counter = 0
        start_node = (AStarPlanner.heuristic(start, goal), counter, start, [start])
        frontier = [start_node]
        visited = set()
        
        # 8-directional movement
        directions = [
            (0, 1), (1, 0), (0, -1), (-1, 0),  # cardinal
            (1, 1), (1, -1), (-1, 1), (-1, -1)  # diagonal
        ]
        
        while frontier:
            f_score, _, current, path = heapq.heappop(frontier)
            
            if current in visited:
                continue
            
            visited.add(current)
            
            # Check if goal reached
            if current == goal:
                return path
            
            # Explore neighbors
            for dx, dy in directions:
                next_pos = (current[0] + dx, current[1] + dy)
                
                if next_pos in visited:
                    continue
                
                if not world.is_valid(next_pos[0], next_pos[1]):
                    continue
                
                # Calculate scores
                g_score = len(path)
                h_score = AStarPlanner.heuristic(next_pos, goal)
                f_score = g_score + h_score
                
                counter += 1
                new_path = path + [next_pos]
                heapq.heappush(frontier, (f_score, counter, next_pos, new_path))
        
        return None  # No path found


class RobotNavigator:
    """High-level robot navigation controller"""
    
    def __init__(self, robot: Robot, world: GridWorld):
        self.robot = robot
        self.world = world
        self.path = []
        self.action_sequence = []
    
    def plan_to_goal(self, goal_x: int, goal_y: int) -> bool:
        """Plan path to goal using A*"""
        start = (int(self.robot.x), int(self.robot.y))
        goal = (goal_x, goal_y)
        
        self.path = AStarPlanner.find_path(self.world, start, goal)
        
        if self.path is None:
            print("No path to goal found!")
            return False
        
        print(f"Path found with {len(self.path)} waypoints")
        return True
    
    def get_target_angle(self, target_x: float, target_y: float) -> float:
        """Calculate angle to target"""
        dx = target_x - self.robot.x
        dy = target_y - self.robot.y
        return math.degrees(math.atan2(dy, dx)) % 360
    
    def navigate_to_goal(self) -> List[str]:
        """Execute navigation plan and return action sequence"""
        if not self.path:
            return []
        
        actions = []
        
        for waypoint in self.path[1:]:  # Skip first waypoint (current position)
            target_x, target_y = waypoint
            
            # Rotate to face target
            while True:
                target_angle = self.get_target_angle(target_x, target_y)
                angle_diff = (target_angle - self.robot.angle) % 360
                
                # Check if already facing target (within tolerance)
                if angle_diff < 15 or angle_diff > 345:
                    break
                
                # Determine rotation direction
                if angle_diff < 180:
                    self.robot.rotate_left()
                    actions.append("ROTATE_LEFT")
                else:
                    self.robot.rotate_right()
                    actions.append("ROTATE_RIGHT")
                
                if len(actions) > 100:  # Safety limit
                    break
            
            # Move toward target
            distance = math.sqrt((target_x - self.robot.x)**2 + (target_y - self.robot.y)**2)
            steps_needed = int(distance / self.robot.move_step) + 1
            
            for _ in range(steps_needed):
                old_pos = (self.robot.x, self.robot.y)
                self.robot.move_forward()
                actions.append("MOVE_FORWARD")
                
                # Check if reached waypoint
                dist_to_waypoint = math.sqrt(
                    (target_x - self.robot.x)**2 + (target_y - self.robot.y)**2
                )
                if dist_to_waypoint < 0.5:
                    break
        
        self.action_sequence = actions
        return actions


class Visualizer:
    """Visualization for robot navigation"""
    
    def __init__(self, world: GridWorld):
        self.world = world
        self.fig, self.ax = plt.subplots(figsize=(12, 10))
    
    def draw_world(self):
        """Draw grid world with obstacles"""
        self.ax.clear()
        self.ax.set_xlim(-1, self.world.width)
        self.ax.set_ylim(-1, self.world.height)
        self.ax.set_aspect('equal')
        self.ax.grid(True, alpha=0.3)
        
        # Draw obstacles
        for obs_x, obs_y, obs_w, obs_h in self.world.obstacles:
            rect = Rectangle((obs_x, obs_y), obs_w, obs_h, 
                           facecolor='gray', edgecolor='black', alpha=0.7)
            self.ax.add_patch(rect)
    
    def draw_robot(self, robot: Robot, color='blue'):
        """Draw robot with FOV indicator"""
        # Robot body
        circle = Circle((robot.x, robot.y), 0.3, color=color, alpha=0.7)
        self.ax.add_patch(circle)
        
        # Direction indicator
        rad = math.radians(robot.angle)
        dx = 0.5 * math.cos(rad)
        dy = 0.5 * math.sin(rad)
        self.ax.arrow(robot.x, robot.y, dx, dy, 
                     head_width=0.2, head_length=0.2, fc=color, ec=color)
        
        # FOV wedge
        start_angle, end_angle = robot.get_fov_bounds()
        wedge = Wedge((robot.x, robot.y), 3, start_angle, end_angle,
                     alpha=0.2, color='yellow')
        self.ax.add_patch(wedge)
    
    def draw_path(self, path: List[Tuple[int, int]], color='green'):
        """Draw planned path"""
        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            self.ax.plot(path_x, path_y, 'o-', color=color, 
                        alpha=0.5, linewidth=2, markersize=4)
    
    def draw_goal(self, goal_x: int, goal_y: int):
        """Draw goal position"""
        circle = Circle((goal_x, goal_y), 0.4, color='red', alpha=0.7)
        self.ax.add_patch(circle)
        self.ax.text(goal_x, goal_y + 0.8, 'GOAL', 
                    ha='center', fontsize=12, fontweight='bold', color='red')
    
    def show(self):
        """Display the visualization"""
        plt.tight_layout()
        plt.show()
    
    def save(self, filename: str):
        """Save visualization to file"""
        plt.tight_layout()
        plt.savefig(filename, dpi=150, bbox_inches='tight')
        print(f"Saved visualization to {filename}")


def main():
    """Main simulation"""
    print("=" * 60)
    print("ROBOT NAVIGATION SYSTEM")
    print("=" * 60)
    
    # Create world
    world = GridWorld(width=20, height=20)
    
    # Add obstacles
    world.add_obstacle(5, 5, 3, 3)
    world.add_obstacle(12, 8, 2, 5)
    world.add_obstacle(8, 15, 4, 2)
    world.add_obstacle(15, 2, 2, 4)
    
    # Create robot
    robot = Robot(x=2.0, y=2.0, angle=45.0)
    print(f"\nRobot initialized at ({robot.x}, {robot.y})")
    print(f"Robot facing: {robot.angle}° (FOV: {robot.fov}°)")
    
    # Set goal
    goal_x, goal_y = 17, 17
    print(f"Goal position: ({goal_x}, {goal_y})")
    
    # Create navigator
    navigator = RobotNavigator(robot, world)
    
    # Plan path
    print("\nPlanning path...")
    if not navigator.plan_to_goal(goal_x, goal_y):
        print("Cannot reach goal!")
        return
    
    # Visualize initial state
    viz = Visualizer(world)
    viz.draw_world()
    viz.draw_path(navigator.path)
    viz.draw_goal(goal_x, goal_y)
    
    # Save initial state
    initial_robot = Robot(x=2.0, y=2.0, angle=45.0)
    viz.draw_robot(initial_robot, color='blue')
    viz.ax.set_title('Initial State - Planned Path', fontsize=14, fontweight='bold')
    viz.save('/mnt/user-data/outputs/robot_navigation_initial.png')
    
    # Execute navigation
    print("\nExecuting navigation...")
    actions = navigator.navigate_to_goal()
    
    print(f"\nNavigation complete!")
    print(f"Total actions: {len(actions)}")
    print(f"Final position: ({robot.x:.2f}, {robot.y:.2f})")
    print(f"Final angle: {robot.angle:.2f}°")
    
    # Count actions
    action_counts = {
        'MOVE_FORWARD': actions.count('MOVE_FORWARD'),
        'ROTATE_LEFT': actions.count('ROTATE_LEFT'),
        'ROTATE_RIGHT': actions.count('ROTATE_RIGHT')
    }
    print(f"\nAction breakdown:")
    for action, count in action_counts.items():
        print(f"  {action}: {count}")
    
    # Visualize final state
    viz2 = Visualizer(world)
    viz2.draw_world()
    viz2.draw_path(navigator.path)
    viz2.draw_goal(goal_x, goal_y)
    viz2.draw_robot(robot, color='green')
    viz2.ax.set_title('Final State - Goal Reached', fontsize=14, fontweight='bold')
    viz2.save('/mnt/user-data/outputs/robot_navigation_final.png')
    
    # Print action sequence (first 30 actions)
    print(f"\nAction sequence (first 30):")
    for i, action in enumerate(actions[:30], 1):
        print(f"{i:3d}. {action}")
    if len(actions) > 30:
        print(f"  ... ({len(actions) - 30} more actions)")
    
    print("\n" + "=" * 60)
    print("Navigation system demonstration complete!")
    print("=" * 60)


if __name__ == "__main__":
    main()
