"""
Advanced Robot Navigation Demo
Multiple scenarios with different complexities
"""

from robot_navigation import Robot, GridWorld, RobotNavigator, Visualizer
import matplotlib.pyplot as plt


def scenario_1_simple():
    """Simple scenario - open space"""
    print("\n" + "="*60)
    print("SCENARIO 1: Simple Navigation - Open Space")
    print("="*60)
    
    world = GridWorld(15, 15)
    robot = Robot(x=2.0, y=2.0, angle=0.0)
    navigator = RobotNavigator(robot, world)
    
    goal_x, goal_y = 12, 12
    
    if navigator.plan_to_goal(goal_x, goal_y):
        viz = Visualizer(world)
        viz.draw_world()
        viz.draw_path(navigator.path)
        viz.draw_goal(goal_x, goal_y)
        
        initial = Robot(x=2.0, y=2.0, angle=0.0)
        viz.draw_robot(initial, color='blue')
        viz.ax.set_title('Scenario 1: Simple Open Space Navigation', fontweight='bold')
        viz.save('/mnt/user-data/outputs/scenario_1_initial.png')
        
        actions = navigator.navigate_to_goal()
        
        viz2 = Visualizer(world)
        viz2.draw_world()
        viz2.draw_path(navigator.path)
        viz2.draw_goal(goal_x, goal_y)
        viz2.draw_robot(robot, color='green')
        viz2.ax.set_title('Scenario 1: Goal Reached', fontweight='bold')
        viz2.save('/mnt/user-data/outputs/scenario_1_final.png')
        
        print(f"Actions: {len(actions)} | Forward: {actions.count('MOVE_FORWARD')} | "
              f"Left: {actions.count('ROTATE_LEFT')} | Right: {actions.count('ROTATE_RIGHT')}")


def scenario_2_maze():
    """Complex maze scenario"""
    print("\n" + "="*60)
    print("SCENARIO 2: Maze Navigation")
    print("="*60)
    
    world = GridWorld(25, 25)
    
    # Create maze-like obstacles
    world.add_obstacle(5, 0, 2, 15)
    world.add_obstacle(10, 5, 2, 20)
    world.add_obstacle(15, 0, 2, 15)
    world.add_obstacle(20, 5, 2, 15)
    world.add_obstacle(0, 10, 8, 2)
    world.add_obstacle(12, 15, 8, 2)
    
    robot = Robot(x=2.0, y=2.0, angle=90.0)
    navigator = RobotNavigator(robot, world)
    
    goal_x, goal_y = 22, 22
    
    if navigator.plan_to_goal(goal_x, goal_y):
        viz = Visualizer(world)
        viz.draw_world()
        viz.draw_path(navigator.path)
        viz.draw_goal(goal_x, goal_y)
        
        initial = Robot(x=2.0, y=2.0, angle=90.0)
        viz.draw_robot(initial, color='blue')
        viz.ax.set_title('Scenario 2: Complex Maze Navigation', fontweight='bold')
        viz.save('/mnt/user-data/outputs/scenario_2_initial.png')
        
        actions = navigator.navigate_to_goal()
        
        viz2 = Visualizer(world)
        viz2.draw_world()
        viz2.draw_path(navigator.path)
        viz2.draw_goal(goal_x, goal_y)
        viz2.draw_robot(robot, color='green')
        viz2.ax.set_title('Scenario 2: Maze Solved', fontweight='bold')
        viz2.save('/mnt/user-data/outputs/scenario_2_final.png')
        
        print(f"Actions: {len(actions)} | Forward: {actions.count('MOVE_FORWARD')} | "
              f"Left: {actions.count('ROTATE_LEFT')} | Right: {actions.count('ROTATE_RIGHT')}")


def scenario_3_narrow_passage():
    """Narrow passage scenario"""
    print("\n" + "="*60)
    print("SCENARIO 3: Narrow Passage Navigation")
    print("="*60)
    
    world = GridWorld(20, 20)
    
    # Create narrow passage
    world.add_obstacle(0, 8, 8, 4)
    world.add_obstacle(12, 8, 8, 4)
    
    robot = Robot(x=2.0, y=2.0, angle=0.0)
    navigator = RobotNavigator(robot, world)
    
    goal_x, goal_y = 17, 17
    
    if navigator.plan_to_goal(goal_x, goal_y):
        viz = Visualizer(world)
        viz.draw_world()
        viz.draw_path(navigator.path)
        viz.draw_goal(goal_x, goal_y)
        
        initial = Robot(x=2.0, y=2.0, angle=0.0)
        viz.draw_robot(initial, color='blue')
        viz.ax.set_title('Scenario 3: Narrow Passage Challenge', fontweight='bold')
        viz.save('/mnt/user-data/outputs/scenario_3_initial.png')
        
        actions = navigator.navigate_to_goal()
        
        viz2 = Visualizer(world)
        viz2.draw_world()
        viz2.draw_path(navigator.path)
        viz2.draw_goal(goal_x, goal_y)
        viz2.draw_robot(robot, color='green')
        viz2.ax.set_title('Scenario 3: Passage Cleared', fontweight='bold')
        viz2.save('/mnt/user-data/outputs/scenario_3_final.png')
        
        print(f"Actions: {len(actions)} | Forward: {actions.count('MOVE_FORWARD')} | "
              f"Left: {actions.count('ROTATE_LEFT')} | Right: {actions.count('ROTATE_RIGHT')}")


def scenario_4_rooms():
    """Multiple rooms scenario"""
    print("\n" + "="*60)
    print("SCENARIO 4: Multi-Room Navigation")
    print("="*60)
    
    world = GridWorld(30, 30)
    
    # Create room-like structure
    world.add_obstacle(0, 10, 10, 2)
    world.add_obstacle(15, 10, 15, 2)
    world.add_obstacle(10, 0, 2, 10)
    world.add_obstacle(10, 12, 2, 18)
    world.add_obstacle(0, 20, 10, 2)
    world.add_obstacle(15, 20, 15, 2)
    
    robot = Robot(x=5.0, y=5.0, angle=45.0)
    navigator = RobotNavigator(robot, world)
    
    goal_x, goal_y = 25, 25
    
    if navigator.plan_to_goal(goal_x, goal_y):
        viz = Visualizer(world)
        viz.draw_world()
        viz.draw_path(navigator.path)
        viz.draw_goal(goal_x, goal_y)
        
        initial = Robot(x=5.0, y=5.0, angle=45.0)
        viz.draw_robot(initial, color='blue')
        viz.ax.set_title('Scenario 4: Multi-Room Environment', fontweight='bold')
        viz.save('/mnt/user-data/outputs/scenario_4_initial.png')
        
        actions = navigator.navigate_to_goal()
        
        viz2 = Visualizer(world)
        viz2.draw_world()
        viz2.draw_path(navigator.path)
        viz2.draw_goal(goal_x, goal_y)
        viz2.draw_robot(robot, color='green')
        viz2.ax.set_title('Scenario 4: Rooms Navigated', fontweight='bold')
        viz2.save('/mnt/user-data/outputs/scenario_4_final.png')
        
        print(f"Actions: {len(actions)} | Forward: {actions.count('MOVE_FORWARD')} | "
              f"Left: {actions.count('ROTATE_LEFT')} | Right: {actions.count('ROTATE_RIGHT')}")


def main():
    """Run all scenarios"""
    print("\n" + "="*70)
    print(" " * 15 + "ROBOT NAVIGATION DEMO SUITE")
    print("="*70)
    
    scenario_1_simple()
    scenario_2_maze()
    scenario_3_narrow_passage()
    scenario_4_rooms()
    
    print("\n" + "="*70)
    print("All scenarios completed! Check the output files.")
    print("="*70)


if __name__ == "__main__":
    main()
