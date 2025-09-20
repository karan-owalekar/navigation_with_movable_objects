#!/usr/bin/env python3
"""
A* Pathfinding Visualizer - Main Application
Author: Karan Owalekar
"""

from visualizer import PathfindingVisualizer

def main():
    """Main application entry point"""
    print("Starting A* Pathfinding Visualizer...")
    print("\nControls:")
    print("- Left click + drag: Add/remove walls/movable obstacles")
    print("- S: Enable start node placement mode (click to place)")
    print("- G: Enable goal node placement mode (click to place)")
    print("- O: Toggle between walls and movable obstacles mode")
    print("- Dropdowns: Select neighbor mode (4/8-way) and heuristic")
    print("- Buttons: Run, Step, Reset, Clear Path")
    print("- SPACE: Find path and start continuous execution")
    print("- ENTER: Step through path execution (only after path is found)")
    print("- +/-: Adjust execution speed")
    print("- ESC or close window: Exit")
    print("\nVisualization:")
    print("- Green transparent: Explored nodes (closed set)")
    print("- Yellow transparent: Open set (being considered)")
    print("- Pink-purple line: Final optimal path")
    print("- Dark gray: Walls/obstacles")
    print("- Light gray with #: Movable obstacles (can be pushed)")
    print("\nSafety Limits:")
    print("- Maximum states explored: 100,000")
    print("- Algorithm will stop if limit exceeded to prevent infinite loops")
    
    # Create and run the visualizer
    try:
        # You can adjust grid_size and cell_size here
        grid_size = 25  # 25x25 grid
        cell_size = 25  # 35 pixels per cell
        
        visualizer = PathfindingVisualizer(grid_size=grid_size, cell_size=cell_size)
        visualizer.run()
        
    except Exception as e:
        print(f"Error starting visualizer: {e}")
        print("Make sure pygame is installed: pip install pygame")

if __name__ == "__main__":
    main()