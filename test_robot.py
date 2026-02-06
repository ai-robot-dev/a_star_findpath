"""
Interactive Robot Testing
Step-by-step visualization of robot movements
"""

from robot_navigation import Robot, GridWorld, RobotNavigator, Visualizer
import matplotlib.pyplot as plt


def test_basic_movements():
    """Test basic robot movements"""
    print("\n" + "="*60)
    print("TEST 1: Basic Movement Capabilities")
    print("="*60)
    
    robot = Robot(x=10.0, y=10.0, angle=0.0)
    
    print(f"\nInitial state:")
    print(f"  Position: ({robot.x:.2f}, {robot.y:.2f})")
    print(f"  Angle: {robot.angle:.2f}°")
    
    print(f"\n1. Move forward:")
    robot.move_forward()
    print(f"  Position: ({robot.x:.2f}, {robot.y:.2f})")
    
    print(f"\n2. Rotate left 30°:")
    robot.rotate_left()
    print(f"  Angle: {robot.angle:.2f}°")
    
    print(f"\n3. Rotate right 30°:")
    robot.rotate_right()
    print(f"  Angle: {robot.angle:.2f}°")
    
    print(f"\n4. Move forward again:")
    robot.move_forward()
    print(f"  Position: ({robot.x:.2f}, {robot.y:.2f})")


def test_fov():
    """Test field of view calculations"""
    print("\n" + "="*60)
    print("TEST 2: Field of View (90 degrees)")
    print("="*60)
    
    robot = Robot(x=10.0, y=10.0, angle=0.0)
    
    test_points = [
        (15, 10, "Directly ahead"),
        (12, 12, "Upper right (45°)"),
        (12, 8, "Lower right (45°)"),
        (10, 15, "Directly above (90°)"),
        (5, 10, "Behind (180°)"),
        (13, 13, "Upper right edge"),
    ]
    
    print(f"\nRobot at ({robot.x}, {robot.y}), facing {robot.angle}°")
    print(f"FOV range: {robot.get_fov_bounds()}")
    print(f"\nChecking visibility of points:")
    
    for px, py, description in test_points:
        visible = robot.can_see_point(px, py)
        status = "✓ VISIBLE" if visible else "✗ NOT VISIBLE"
        print(f"  ({px}, {py}) - {description:20s}: {status}")


def test_pathfinding():
    """Test A* pathfinding with obstacles"""
    print("\n" + "="*60)
    print("TEST 3: A* Pathfinding with Obstacles")
    print("="*60)
    
    world = GridWorld(15, 15)
    world.add_obstacle(6, 5, 3, 5)
    
    robot = Robot(x=2.0, y=2.0, angle=0.0)
    navigator = RobotNavigator(robot, world)
    
    goal_x, goal_y = 12, 12
    
    print(f"\nWorld: 15x15 grid")
    print(f"Obstacle: (6, 5) size 3x5")
    print(f"Start: ({robot.x}, {robot.y})")
    print(f"Goal: ({goal_x}, {goal_y})")
    
    success = navigator.plan_to_goal(goal_x, goal_y)
    
    if success:
        print(f"\n✓ Path found!")
        print(f"  Waypoints: {len(navigator.path)}")
        print(f"  Path: {navigator.path[:5]}...")
        
        # Visualize
        viz = Visualizer(world)
        viz.draw_world()
        viz.draw_path(navigator.path)
        viz.draw_goal(goal_x, goal_y)
        viz.draw_robot(robot, color='blue')
        viz.ax.set_title('Pathfinding Test - A* Algorithm', fontweight='bold')
        viz.save('/mnt/user-data/outputs/test_pathfinding.png')
        print(f"\n  Visualization saved!")
    else:
        print(f"\n✗ No path found!")


def test_rotation_strategy():
    """Test rotation direction selection"""
    print("\n" + "="*60)
    print("TEST 4: Rotation Strategy")
    print("="*60)
    
    robot = Robot(x=10.0, y=10.0, angle=0.0)
    
    test_targets = [
        (15, 10, 0, "Right"),
        (10, 15, 90, "Up"),
        (5, 10, 180, "Left"),
        (10, 5, 270, "Down"),
        (15, 15, 45, "Upper Right"),
        (5, 5, 225, "Lower Left"),
    ]
    
    print(f"\nRobot at ({robot.x}, {robot.y})")
    print(f"\nTesting rotation to various targets:")
    
    for tx, ty, expected_angle, direction in test_targets:
        robot.angle = 0.0  # Reset
        navigator = RobotNavigator(robot, None)
        target_angle = navigator.get_target_angle(tx, ty)
        angle_diff = (target_angle - robot.angle) % 360
        
        if angle_diff < 180:
            rotation = "LEFT (counter-clockwise)"
        else:
            rotation = "RIGHT (clockwise)"
        
        print(f"  Target ({tx:2d}, {ty:2d}) - {direction:12s}: "
              f"{target_angle:6.1f}° -> Rotate {rotation}")


def test_edge_cases():
    """Test edge cases and special scenarios"""
    print("\n" + "="*60)
    print("TEST 5: Edge Cases")
    print("="*60)
    
    # Test 1: Goal at current position
    print("\n1. Goal at current position:")
    world = GridWorld(10, 10)
    robot = Robot(x=5.0, y=5.0, angle=0.0)
    navigator = RobotNavigator(robot, world)
    
    if navigator.plan_to_goal(5, 5):
        print(f"   Path length: {len(navigator.path)}")
    
    # Test 2: Adjacent goal
    print("\n2. Adjacent goal:")
    robot = Robot(x=5.0, y=5.0, angle=0.0)
    navigator = RobotNavigator(robot, world)
    
    if navigator.plan_to_goal(6, 5):
        print(f"   Path length: {len(navigator.path)}")
    
    # Test 3: Blocked goal
    print("\n3. Completely blocked goal:")
    world2 = GridWorld(10, 10)
    world2.add_obstacle(7, 7, 3, 3)  # Block goal area
    robot = Robot(x=2.0, y=2.0, angle=0.0)
    navigator = RobotNavigator(robot, world2)
    
    if navigator.plan_to_goal(8, 8):
        print(f"   Path found: {len(navigator.path)} waypoints")
    else:
        print(f"   ✓ Correctly detected: No path available")


def test_performance():
    """Test performance on large grid"""
    print("\n" + "="*60)
    print("TEST 6: Performance on Large Grid")
    print("="*60)
    
    import time
    
    world = GridWorld(50, 50)
    
    # Add random obstacles
    import random
    random.seed(42)
    for _ in range(20):
        x = random.randint(0, 45)
        y = random.randint(0, 45)
        w = random.randint(1, 4)
        h = random.randint(1, 4)
        world.add_obstacle(x, y, w, h)
    
    robot = Robot(x=2.0, y=2.0, angle=0.0)
    navigator = RobotNavigator(robot, world)
    
    print(f"\nGrid size: 50x50")
    print(f"Obstacles: 20 random obstacles")
    print(f"Start: (2, 2)")
    print(f"Goal: (47, 47)")
    
    start_time = time.time()
    success = navigator.plan_to_goal(47, 47)
    planning_time = time.time() - start_time
    
    if success:
        print(f"\n✓ Path found!")
        print(f"  Planning time: {planning_time*1000:.2f} ms")
        print(f"  Path length: {len(navigator.path)} waypoints")
        
        start_time = time.time()
        actions = navigator.navigate_to_goal()
        #Don't use actions
        print(navigator.path)
        execution_time = time.time() - start_time
        
        print(f"  Execution time: {execution_time*1000:.2f} ms")
        print(f"  Total actions: {len(actions)}")


def main():
    """Run all tests"""
    print("\n" + "="*70)
    print(" " * 20 + "ROBOT NAVIGATION TESTS")
    print("="*70)
    
    test_basic_movements()
    test_fov()
    test_pathfinding()
    test_rotation_strategy()
    test_edge_cases()
    test_performance()
    
    print("\n" + "="*70)
    print("All tests completed!")
    print("="*70)


if __name__ == "__main__":
    main()
