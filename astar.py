"""
A* Pathfinding Algorithm Implementation with Push Mechanics
"""
import heapq
from typing import List, Optional, Set, Tuple, Callable, Dict
from grid import Grid, Node, NodeType
import copy
from config import NORMAL_MOVE_COST, DIAGONAL_MOVE_COST, PUSH_COST

class WorldState:
    """Represents a complete world state including planner position and movable obstacles"""
    
    def __init__(self, planner_pos: Tuple[int, int], movable_obstacles: Dict[Tuple[int, int], bool]):
        self.planner_pos = planner_pos  # (row, col) of planner
        self.movable_obstacles = movable_obstacles.copy()  # Dict of (row, col) -> True for movable obstacles
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.parent_state: Optional['WorldState'] = None
        self.action = None  # The action that led to this state
        
    def __hash__(self):
        """Make WorldState hashable for sets"""
        obstacles_tuple = tuple(sorted(self.movable_obstacles.items()))
        return hash((self.planner_pos, obstacles_tuple))
    
    def __eq__(self, other):
        """Check equality for sets"""
        if not isinstance(other, WorldState):
            return False
        return (self.planner_pos == other.planner_pos and 
                self.movable_obstacles == other.movable_obstacles)
    
    def __lt__(self, other):
        """For priority queue comparison"""
        if self.f_cost == other.f_cost:
            return self.h_cost < other.h_cost
        return self.f_cost < other.f_cost
    
    def calculate_f_cost(self):
        """Calculate total cost"""
        self.f_cost = self.g_cost + self.h_cost


class AStarPathfinder:
    def __init__(self, grid: Grid):
        self.grid = grid
        self.open_set: List[Node] = []
        self.open_set_hash: Set[Tuple[int, int]] = set()
        self.closed_set: Set[Tuple[int, int]] = set()
        self.path: List[Node] = []
        self.current_node: Optional[Node] = None
        
        # Push mechanics support
        self.push_enabled = True  # Enable push mechanics
        self.world_states: List[WorldState] = []  # For state-space search
        self.world_open_set: Set[WorldState] = set()
        self.world_closed_set: Set[WorldState] = set()
        self.current_world_state: Optional[WorldState] = None
        self.initial_movable_obstacles: Dict[Tuple[int, int], bool] = {}
        
        # Safety limits
        self.max_states_explored = 100000
        self.states_explored = 0
        
        # Callback for visualization updates
        self.visualization_callback: Optional[Callable] = None
        
    def set_visualization_callback(self, callback: Callable):
        """Set callback function for visualization updates"""
        self.visualization_callback = callback
        
    def reset(self):
        """Reset algorithm state"""
        self.open_set.clear()
        self.open_set_hash.clear()
        self.closed_set.clear()
        self.path.clear()
        self.current_node = None
        self.world_states.clear()
        self.world_open_set.clear()
        self.world_closed_set.clear()
        self.current_world_state = None
        self.grid.clear_path_visualization()
        
        # Restore initial obstacle positions if they were captured
        if self.initial_movable_obstacles:
            self.grid.set_movable_obstacles(self.initial_movable_obstacles)
        
        # Reset safety counter
        self.states_explored = 0
        
    def get_movable_obstacles(self) -> Dict[Tuple[int, int], bool]:
        """Get current positions of all movable obstacles"""
        obstacles = {}
        for row in range(self.grid.size):
            for col in range(self.grid.size):
                node = self.grid.get_node(row, col)
                if node and node.node_type == NodeType.MOVABLE_OBSTACLE:
                    obstacles[(row, col)] = True
        return obstacles
    
    def can_push_obstacles(self, from_pos: Tuple[int, int], to_pos: Tuple[int, int], 
                          world_state: WorldState) -> Tuple[bool, int, Dict[Tuple[int, int], bool]]:
        """
        Check if we can push obstacles from from_pos to to_pos
        
        Returns:
            (can_push, cost, new_obstacle_positions)
        """
        # First check if the destination is within bounds
        if (to_pos[0] < 0 or to_pos[0] >= self.grid.size or 
            to_pos[1] < 0 or to_pos[1] >= self.grid.size):
            return False, float('inf'), {}
        
        # Check if destination is a wall (cannot move into walls)
        dest_node = self.grid.get_node(*to_pos)
        if not dest_node or dest_node.node_type == NodeType.WALL:
            return False, float('inf'), {}
        
        # If destination has no movable obstacle, it's a normal move
        if to_pos not in world_state.movable_obstacles:
            # Check if destination is start or goal (only goal is allowed)
            if dest_node.node_type == NodeType.START:
                return False, float('inf'), {}
            
            # Determine if move is diagonal
            is_diagonal = abs(from_pos[0] - to_pos[0]) == 1 and abs(from_pos[1] - to_pos[1]) == 1
            move_cost = DIAGONAL_MOVE_COST if is_diagonal else NORMAL_MOVE_COST
            return True, move_cost, world_state.movable_obstacles
        
        # Calculate direction of push
        dr = to_pos[0] - from_pos[0]
        dc = to_pos[1] - from_pos[1]
        
        # Find all obstacles in the push chain
        push_chain = []
        current_pos = to_pos
        
        while current_pos in world_state.movable_obstacles:
            push_chain.append(current_pos)
            current_pos = (current_pos[0] + dr, current_pos[1] + dc)
            
            # Check bounds
            if (current_pos[0] < 0 or current_pos[0] >= self.grid.size or 
                current_pos[1] < 0 or current_pos[1] >= self.grid.size):
                return False, float('inf'), {}
            
            # Check if pushing into a wall
            node = self.grid.get_node(*current_pos)
            if node and node.node_type == NodeType.WALL:
                return False, float('inf'), {}
        
        # Check if final position is valid (empty or goal)
        final_node = self.grid.get_node(*current_pos)
        if not final_node or final_node.node_type in [NodeType.WALL, NodeType.START]:
            return False, float('inf'), {}
        
        # Can't push obstacles onto goal unless it's the final step
        if final_node.node_type == NodeType.GOAL and len(push_chain) > 0:
            return False, float('inf'), {}
        
        # Calculate new obstacle positions after push
        new_obstacles = world_state.movable_obstacles.copy()
        
        # Remove obstacles from their current positions
        for pos in push_chain:
            del new_obstacles[pos]
        
        # Add obstacles to their new positions
        for i, old_pos in enumerate(push_chain):
            new_pos = (old_pos[0] + dr, old_pos[1] + dc)
            new_obstacles[new_pos] = True
        
        # Cost is move cost + push_cost for each object pushed
        is_diagonal = abs(from_pos[0] - to_pos[0]) == 1 and abs(from_pos[1] - to_pos[1]) == 1
        move_cost = DIAGONAL_MOVE_COST if is_diagonal else NORMAL_MOVE_COST
        total_cost = move_cost + (len(push_chain) * PUSH_COST)
        
        return True, total_cost, new_obstacles
    
    def get_next_world_states(self, world_state: WorldState) -> List[Tuple[WorldState, int]]:
        """
        Get all possible next world states from current state
        
        Returns:
            List of (new_world_state, move_cost) tuples
        """
        next_states = []
        current_row, current_col = world_state.planner_pos
        
        # Get neighbors based on movement mode
        neighbors = self.grid.get_neighbor_coords(current_row, current_col)
        
        for neighbor_row, neighbor_col in neighbors:
            # Check if this move is valid (considering pushes)
            can_move, cost, new_obstacles = self.can_push_obstacles(
                world_state.planner_pos, 
                (neighbor_row, neighbor_col), 
                world_state
            )
            
            if can_move and cost < float('inf'):
                # Create new world state
                new_state = WorldState(
                    planner_pos=(neighbor_row, neighbor_col),
                    movable_obstacles=new_obstacles
                )
                next_states.append((new_state, cost))
            # Debug: uncomment to see rejected moves
            # else:
            #     print(f"Rejected move to ({neighbor_row}, {neighbor_col}): cost={cost}")
        
        return next_states
    
    def find_path_with_pushes(self, step_by_step: bool = False) -> Optional[List[WorldState]]:
        """
        Find path using A* with push mechanics (state-space search)
        
        Args:
            step_by_step: If True, returns after each step for visualization
            
        Returns:
            Complete path of world states if found, None if no path exists
        """
        if not self.grid.start_pos or not self.grid.goal_pos:
            return None
        
        # For push mechanics, we always plan completely in background
        # No step-by-step visualization during planning
        
        # Create initial world state
        self.initial_movable_obstacles = self.get_movable_obstacles()
        start_state = WorldState(
            planner_pos=self.grid.start_pos,
            movable_obstacles=self.initial_movable_obstacles
        )
        start_state.g_cost = 0
        start_state.h_cost = self.grid.heuristic_coords(
            self.grid.start_pos[0], self.grid.start_pos[1],
            self.grid.goal_pos[0], self.grid.goal_pos[1]
        )
        start_state.calculate_f_cost()
        
        # Initialize search
        self.world_states = [start_state]
        self.world_open_set = {start_state}
        self.world_closed_set = set()
        self.current_world_state = start_state
        
        # Reset states explored counter
        self.states_explored = 0
        
        print(f"Starting push search from {self.grid.start_pos} to {self.grid.goal_pos}")
        print(f"Initial obstacles: {list(self.initial_movable_obstacles.keys())}")
        
        while self.world_states and self.states_explored < self.max_states_explored:
            self.states_explored += 1
            
            # Print progress every 1000 states
            if self.states_explored % 1000 == 0:
                print(f"Search step {self.states_explored}, open states: {len(self.world_open_set)}")
            
            # Get state with lowest f_cost
            current_state = heapq.heappop(self.world_states)
            self.current_world_state = current_state
            
            # Remove from open set and add to closed set
            self.world_open_set.discard(current_state)
            self.world_closed_set.add(current_state)
            
            # Check if we reached the goal
            if current_state.planner_pos == self.grid.goal_pos:
                print(f"Path found in {self.states_explored} steps!")
                # Reconstruct path
                path = []
                state = current_state
                while state:
                    path.append(state)
                    state = state.parent_state
                path.reverse()
                print(f"Final path has {len(path)} steps")
                return path
            
            # Get all possible next states
            next_states = self.get_next_world_states(current_state)
            
            for next_state, move_cost in next_states:
                # Skip if already in closed set
                if next_state in self.world_closed_set:
                    continue
                
                # Calculate costs
                tentative_g_cost = current_state.g_cost + move_cost
                
                # Check if this is a better path to next_state
                if next_state not in self.world_open_set:
                    # New state
                    next_state.g_cost = tentative_g_cost
                    next_state.h_cost = self.grid.heuristic_coords(
                        next_state.planner_pos[0], next_state.planner_pos[1],
                        self.grid.goal_pos[0], self.grid.goal_pos[1]
                    )
                    next_state.calculate_f_cost()
                    next_state.parent_state = current_state
                    
                    heapq.heappush(self.world_states, next_state)
                    self.world_open_set.add(next_state)
                    
                elif tentative_g_cost < next_state.g_cost:
                    # Better path found
                    next_state.g_cost = tentative_g_cost
                    next_state.calculate_f_cost()
                    next_state.parent_state = current_state
                    
                    # Re-heapify since we changed the cost
                    heapq.heapify(self.world_states)
        
        # Check if we exceeded the limit
        if self.states_explored >= self.max_states_explored:
            print(f"Search stopped: Exceeded maximum states limit ({self.max_states_explored})")
            print("Try simplifying the problem or reducing the number of movable obstacles")
            return None
        
        print("No path found!")
        return None  # No path found
    
    def apply_world_state_to_grid(self, world_state: WorldState):
        """Apply world state to grid for visualization (temporarily)"""
        # This is for visualization only - doesn't modify the actual grid permanently
        
        # Clear current movable obstacles
        for row in range(self.grid.size):
            for col in range(self.grid.size):
                node = self.grid.get_node(row, col)
                if node and node.node_type == NodeType.MOVABLE_OBSTACLE:
                    node.node_type = NodeType.EMPTY
        
        # Place obstacles in new positions
        for (row, col) in world_state.movable_obstacles:
            node = self.grid.get_node(row, col)
            if node:
                node.node_type = NodeType.MOVABLE_OBSTACLE
        
        # Mark planner position
        planner_node = self.grid.get_node(*world_state.planner_pos)
        if planner_node:
            planner_node.node_type = NodeType.CURRENT
        
    def reconstruct_path(self, end_node: Node) -> List[Node]:
        """Reconstruct path from end node to start"""
        path = []
        current = end_node
        
        while current:
            path.append(current)
            current = current.parent
            
        path.reverse()
        return path
        
    def find_path(self, step_by_step: bool = False) -> Optional[List[Node]]:
        """
        Find path using A* algorithm with automatic push mechanics detection
        
        Args:
            step_by_step: If True, returns after each step for visualization
            
        Returns:
            Complete path if found, None if no path exists
        """
        if not self.grid.start_pos or not self.grid.goal_pos:
            return None
        
        # Check if there are any movable obstacles
        movable_obstacles = self.get_movable_obstacles()
        
        print(f"Debug: Found {len(movable_obstacles)} movable obstacles")
        print(f"Debug: Push enabled: {self.push_enabled}")
        
        if self.push_enabled and movable_obstacles:
            print("Debug: Using push mechanics (state-space search)")
            # Use state-space search with push mechanics
            world_path = self.find_path_with_pushes(step_by_step)
            if world_path:
                # Convert world state path to node path for visualization
                return self.convert_world_path_to_nodes(world_path)
            return None
        else:
            print("Debug: Using traditional A* pathfinding")
            # Use traditional A* pathfinding
            return self._find_path_traditional(step_by_step)
    
    def convert_world_path_to_nodes(self, world_path: List[WorldState]) -> List[Node]:
        """Convert a world state path to a node path for visualization"""
        node_path = []
        for world_state in world_path:
            row, col = world_state.planner_pos
            node = self.grid.get_node(row, col)
            if node:
                node_path.append(node)
        
        # Store the world path for step-by-step execution
        self.world_path = world_path
        self.world_path_index = 0
        print(f"Path stored for execution: {len(world_path)} states")
        return node_path
    
    def execute_next_world_step(self) -> bool:
        """Execute the next step in the world path"""
        if not hasattr(self, 'world_path') or not self.world_path:
            return False
            
        if self.world_path_index < len(self.world_path):
            world_state = self.world_path[self.world_path_index]
            self.apply_world_state_to_grid(world_state)
            self.world_path_index += 1
            print(f"Executing step {self.world_path_index}/{len(self.world_path)}")
            
            if self.visualization_callback:
                self.visualization_callback()
            
            return self.world_path_index < len(self.world_path)
        
        return False
    
    def reset_world_execution(self):
        """Reset world path execution to beginning"""
        if hasattr(self, 'world_path'):
            self.world_path_index = 0
    
    def _find_path_traditional(self, step_by_step: bool = False) -> Optional[List[Node]]:
        """Traditional A* pathfinding without push mechanics"""
            
        start_node = self.grid.get_node(*self.grid.start_pos)
        goal_node = self.grid.get_node(*self.grid.goal_pos)
        
        if not start_node or not goal_node:
            return None
            
        # Initialize start node
        start_node.g_cost = 0
        start_node.h_cost = self.grid.heuristic(start_node, goal_node)
        start_node.calculate_f_cost()
        
        # Add start node to open set
        heapq.heappush(self.open_set, start_node)
        self.open_set_hash.add((start_node.row, start_node.col))
        
        while self.open_set:
            # Get node with lowest f_cost
            self.current_node = heapq.heappop(self.open_set)
            current_pos = (self.current_node.row, self.current_node.col)
            self.open_set_hash.discard(current_pos)
            
            # Mark as current for visualization
            if self.current_node.node_type not in [NodeType.START, NodeType.GOAL]:
                self.current_node.node_type = NodeType.CURRENT
                
            # Update visualization
            if self.visualization_callback:
                self.visualization_callback()
                
            # Check if we reached the goal
            if current_pos == self.grid.goal_pos:
                self.path = self.reconstruct_path(self.current_node)
                self._mark_path()
                return self.path
                
            # Move current node to closed set
            self.closed_set.add(current_pos)
            if self.current_node.node_type == NodeType.CURRENT:
                self.current_node.node_type = NodeType.CLOSED
                
            # Check all neighbors
            for neighbor in self.grid.get_neighbors(self.current_node):
                neighbor_pos = (neighbor.row, neighbor.col)
                
                # Skip if already in closed set
                if neighbor_pos in self.closed_set:
                    continue
                    
                # Calculate tentative g_cost
                move_cost = self.grid.distance(self.current_node, neighbor)
                tentative_g_cost = self.current_node.g_cost + move_cost
                
                # Check if this path is better
                if tentative_g_cost < neighbor.g_cost:
                    neighbor.parent = self.current_node
                    neighbor.g_cost = tentative_g_cost
                    neighbor.h_cost = self.grid.heuristic(neighbor, goal_node)
                    neighbor.calculate_f_cost()
                    
                    # Add to open set if not already there
                    if neighbor_pos not in self.open_set_hash:
                        heapq.heappush(self.open_set, neighbor)
                        self.open_set_hash.add(neighbor_pos)
                        
                        # Mark for visualization
                        if neighbor.node_type not in [NodeType.START, NodeType.GOAL]:
                            neighbor.node_type = NodeType.OPEN
                            
            if step_by_step:
                return None  # Return None to indicate step completed but path not found yet
                
        return None  # No path found
        
    def step(self) -> Optional[bool]:
        """
        Perform one step of the A* algorithm
        
        Returns:
            True if path found, False if step completed, None if no more steps possible
        """
        if not self.grid.start_pos or not self.grid.goal_pos:
            return None
            
        # Initialize if this is the first step
        if not self.open_set and not self.closed_set:
            start_node = self.grid.get_node(*self.grid.start_pos)
            goal_node = self.grid.get_node(*self.grid.goal_pos)
            
            if not start_node or not goal_node:
                return None
                
            # Initialize start node
            start_node.g_cost = 0
            start_node.h_cost = self.grid.heuristic(start_node, goal_node)
            start_node.calculate_f_cost()
            
            # Add start node to open set
            heapq.heappush(self.open_set, start_node)
            self.open_set_hash.add((start_node.row, start_node.col))
        
        if not self.open_set:
            return None  # No more steps possible
            
        goal_node = self.grid.get_node(*self.grid.goal_pos)
        
        # Get node with lowest f_cost
        self.current_node = heapq.heappop(self.open_set)
        current_pos = (self.current_node.row, self.current_node.col)
        self.open_set_hash.discard(current_pos)
        
        # Mark as current for visualization
        if self.current_node.node_type not in [NodeType.START, NodeType.GOAL]:
            self.current_node.node_type = NodeType.CURRENT
            
        # Update visualization
        if self.visualization_callback:
            self.visualization_callback()
            
        # Check if we reached the goal
        if current_pos == self.grid.goal_pos:
            self.path = self.reconstruct_path(self.current_node)
            self._mark_path()
            return True  # Path found
            
        # Move current node to closed set
        self.closed_set.add(current_pos)
        if self.current_node.node_type == NodeType.CURRENT:
            self.current_node.node_type = NodeType.CLOSED
            
        # Check all neighbors
        for neighbor in self.grid.get_neighbors(self.current_node):
            neighbor_pos = (neighbor.row, neighbor.col)
            
            # Skip if already in closed set
            if neighbor_pos in self.closed_set:
                continue
                
            # Calculate tentative g_cost
            tentative_g_cost = (self.current_node.g_cost + 
                              self.grid.distance(self.current_node, neighbor))
            
            # Check if this path is better
            if tentative_g_cost < neighbor.g_cost:
                neighbor.parent = self.current_node
                neighbor.g_cost = tentative_g_cost
                neighbor.h_cost = self.grid.heuristic(neighbor, goal_node)
                neighbor.calculate_f_cost()
                
                # Add to open set if not already there
                if neighbor_pos not in self.open_set_hash:
                    heapq.heappush(self.open_set, neighbor)
                    self.open_set_hash.add(neighbor_pos)
                    
                    # Mark for visualization
                    if neighbor.node_type not in [NodeType.START, NodeType.GOAL]:
                        neighbor.node_type = NodeType.OPEN
                        
        return False  # Step completed, continue
            
    def _mark_path(self):
        """Mark the final path for visualization"""
        for node in self.path:
            if node.node_type not in [NodeType.START, NodeType.GOAL]:
                node.node_type = NodeType.PATH
                
    def get_open_nodes(self) -> List[Node]:
        """Get list of nodes currently in open set"""
        return [node for node in self.open_set if (node.row, node.col) in self.open_set_hash]
        
    def get_closed_nodes(self) -> List[Tuple[int, int]]:
        """Get list of positions in closed set"""
        return list(self.closed_set)
        
    def get_current_node(self) -> Optional[Node]:
        """Get currently processing node"""
        return self.current_node