# A* Pathfinding Visualizer

An interactive Python application that visualizes the A* pathfinding algorithm on a grid. You can create obstacles, set start and goal points, and watch the algorithm find the optimal path in real-time.

## Features

- **Interactive Grid**: Click to add/remove walls, drag start and goal points
- **Real-time Visualization**: See the algorithm's progress with color-coded nodes
- **Queue Visualization**: Watch open and closed sets as the algorithm explores
- **Adjustable Speed**: Control how fast the algorithm runs
- **Path Reconstruction**: See the final optimal path highlighted

## Installation

1. Make sure you have Python 3.6+ installed
2. Install required packages:
   ```bash
   pip install pygame numpy
   ```

## Usage

Run the application:
```bash
python main.py
```

### Controls

- **Left Click**: Add/remove walls (black squares)
- **Right Click**: Move start point (green square)
- **Middle Click**: Move goal point (red square)
- **SPACE**: Start/stop the pathfinding algorithm
- **R**: Reset the entire grid
- **C**: Clear path visualization only (keep walls)
- **+/-**: Increase/decrease algorithm speed
- **ESC** or close window: Exit

### Visual Legend

- **White**: Empty space
- **Black**: Walls/obstacles
- **Green**: Start point
- **Red**: Goal point
- **Cyan**: Open set (nodes being considered by the algorithm)
- **Gray**: Closed set (nodes already processed)
- **Magenta**: Current node being processed
- **Yellow**: Final optimal path

## How A* Works

The A* algorithm finds the shortest path between two points by:

1. **Maintaining two sets**:
   - Open set: Nodes to be evaluated
   - Closed set: Nodes already evaluated

2. **Using cost functions**:
   - G-cost: Distance from start node
   - H-cost: Heuristic distance to goal (straight-line distance)
   - F-cost: G-cost + H-cost (total estimated cost)

3. **Algorithm steps**:
   - Start with the initial node in the open set
   - Repeatedly pick the node with lowest F-cost from open set
   - Move it to closed set and examine its neighbors
   - Update neighbor costs if a better path is found
   - Continue until goal is reached or no path exists

## File Structure

- `main.py`: Entry point and application setup
- `visualizer.py`: Pygame-based visualization and user interface
- `astar.py`: A* pathfinding algorithm implementation
- `grid.py`: Grid and node data structures

## Customization

You can customize the application by modifying parameters in `main.py`:

- `grid_size`: Size of the grid (default: 25x25)
- `cell_size`: Size of each cell in pixels (default: 25)

## Algorithm Complexity

- **Time Complexity**: O(b^d) where b is the branching factor and d is the depth
- **Space Complexity**: O(b^d) for storing the open and closed sets
- **Optimality**: A* is optimal if the heuristic is admissible (never overestimates)

## Tips for Use

1. **Start Simple**: Begin with a clear path, then add obstacles
2. **Watch the Colors**: The cyan nodes show what the algorithm is considering
3. **Try Different Layouts**: Create mazes or complex obstacle patterns
4. **Adjust Speed**: Use +/- keys to see the algorithm in slow motion or speed it up
5. **Reset Often**: Use 'R' to clear everything or 'C' to just clear the path

Enjoy exploring pathfinding algorithms!