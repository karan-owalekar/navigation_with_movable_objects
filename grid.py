"""
Grid and Node classes for A* pathfinding visualization
"""
from typing import Tuple, List, Optional
from enum import Enum
from config import NORMAL_MOVE_COST, DIAGONAL_MOVE_COST


class NodeType(Enum):
    EMPTY = 0
    WALL = 1
    START = 2
    GOAL = 3
    PATH = 4
    OPEN = 5      # In open set (being considered)
    CLOSED = 6    # In closed set (already processed)
    CURRENT = 7   # Currently being processed
    MOVABLE_OBSTACLE = 8  # Movable obstacle that can be pushed

class Node:
    def __init__(self, row: int, col: int, node_type: NodeType = NodeType.EMPTY):
        self.row = row
        self.col = col
        self.node_type = node_type
        
        # A* algorithm properties
        self.g_cost = float('inf')  # Distance from start
        self.h_cost = 0             # Heuristic (distance to goal)
        self.f_cost = float('inf')  # Total cost (g + h)
        self.parent: Optional['Node'] = None
        
    def reset_costs(self):
        """Reset A* costs for new search"""
        self.g_cost = float('inf')
        self.h_cost = 0
        self.f_cost = float('inf')
        self.parent = None
        
    def calculate_f_cost(self):
        """Calculate total cost"""
        self.f_cost = self.g_cost + self.h_cost
        
    def __lt__(self, other):
        """For priority queue comparison"""
        if self.f_cost == other.f_cost:
            return self.h_cost < other.h_cost
        return self.f_cost < other.f_cost

class Grid:
    def __init__(self, size: int):
        self.size = size
        self.nodes = [[Node(row, col) for col in range(size)] for row in range(size)]
        self.start_pos: Optional[Tuple[int, int]] = None
        self.goal_pos: Optional[Tuple[int, int]] = None
        
        # Algorithm configuration
        self.neighbor_mode = 4  # 4 or 8 neighbors
        self.heuristic_mode = "manhattan"  # "manhattan", "euclidean", "octile"
        
    def get_node(self, row: int, col: int) -> Optional[Node]:
        """Get node at position if valid"""
        if 0 <= row < self.size and 0 <= col < self.size:
            return self.nodes[row][col]
        return None
        
    def set_start(self, row: int, col: int):
        """Set start position"""
        # Clear previous start
        if self.start_pos:
            old_row, old_col = self.start_pos
            if self.nodes[old_row][old_col].node_type == NodeType.START:
                self.nodes[old_row][old_col].node_type = NodeType.EMPTY
                
        self.start_pos = (row, col)
        self.nodes[row][col].node_type = NodeType.START
        
    def set_goal(self, row: int, col: int):
        """Set goal position"""
        # Clear previous goal
        if self.goal_pos:
            old_row, old_col = self.goal_pos
            if self.nodes[old_row][old_col].node_type == NodeType.GOAL:
                self.nodes[old_row][old_col].node_type = NodeType.EMPTY
                
        self.goal_pos = (row, col)
        self.nodes[row][col].node_type = NodeType.GOAL
        
    def toggle_wall(self, row: int, col: int):
        """Toggle wall at position"""
        node = self.get_node(row, col)
        if node and node.node_type not in [NodeType.START, NodeType.GOAL]:
            if node.node_type == NodeType.WALL:
                node.node_type = NodeType.EMPTY
            else:
                node.node_type = NodeType.WALL
                
    def clear_path_visualization(self):
        """Clear path, open, closed, and current node visualizations"""
        for row in self.nodes:
            for node in row:
                if node.node_type in [NodeType.PATH, NodeType.OPEN, NodeType.CLOSED, NodeType.CURRENT]:
                    node.node_type = NodeType.EMPTY
                node.reset_costs()
                
    def get_neighbors(self, node: Node) -> List[Node]:
        """Get valid neighbors based on neighbor mode"""
        neighbors = []
        
        if self.neighbor_mode == 4:
            # 4-directional movement (up, down, left, right)
            directions = [
                (-1, 0), (1, 0),   # Up, Down
                (0, -1), (0, 1)    # Left, Right
            ]
        else:
            # 8-directional movement
            directions = [
                (-1, -1), (-1, 0), (-1, 1),  # Top row
                (0, -1),           (0, 1),   # Middle row (excluding center)
                (1, -1),  (1, 0),  (1, 1)    # Bottom row
            ]
        
        for dr, dc in directions:
            new_row, new_col = node.row + dr, node.col + dc
            neighbor = self.get_node(new_row, new_col)
            
            if neighbor and neighbor.node_type not in [NodeType.WALL, NodeType.MOVABLE_OBSTACLE]:
                neighbors.append(neighbor)
                
        return neighbors
        
    def get_neighbor_coords(self, row: int, col: int) -> List[Tuple[int, int]]:
        """Get valid neighbor coordinates based on neighbor mode"""
        neighbors = []
        
        if self.neighbor_mode == 4:
            # 4-directional movement (up, down, left, right)
            directions = [
                (-1, 0), (1, 0),   # Up, Down
                (0, -1), (0, 1)    # Left, Right
            ]
        else:
            # 8-directional movement
            directions = [
                (-1, -1), (-1, 0), (-1, 1),  # Top row
                (0, -1),           (0, 1),   # Middle row (excluding center)
                (1, -1),  (1, 0),  (1, 1)    # Bottom row
            ]
        
        for dr, dc in directions:
            new_row, new_col = row + dr, col + dc
            
            # Check bounds
            if 0 <= new_row < self.size and 0 <= new_col < self.size:
                neighbors.append((new_row, new_col))
                
        return neighbors
        
    def heuristic(self, node1: Node, node2: Node) -> float:
        """Calculate heuristic distance between two nodes based on heuristic mode"""
        return self.heuristic_coords(node1.row, node1.col, node2.row, node2.col)
        
    def heuristic_coords(self, row1: int, col1: int, row2: int, col2: int) -> float:
        """Calculate heuristic distance between two coordinates based on heuristic mode"""
        dx = abs(row1 - row2)
        dy = abs(col1 - col2)
        
        if self.heuristic_mode == "manhattan":
            return (dx + dy) * NORMAL_MOVE_COST
        elif self.heuristic_mode == "euclidean":
            return (((dx) ** 2 + (dy) ** 2) ** 0.5) * NORMAL_MOVE_COST
        elif self.heuristic_mode == "octile":
            # Octile distance (good for 8-directional movement)
            return (max(dx, dy) + (DIAGONAL_MOVE_COST - 1) * min(dx, dy)) * NORMAL_MOVE_COST
        else:
            # Default to Manhattan
            return (dx + dy) * NORMAL_MOVE_COST
        
    def distance(self, node1: Node, node2: Node) -> float:
        """Calculate actual distance between two nodes"""
        dx = abs(node1.row - node2.row)
        dy = abs(node1.col - node2.col)
        
        # For diagonal movement, use appropriate distance calculation
        if self.neighbor_mode == 8 and dx == 1 and dy == 1:
            return DIAGONAL_MOVE_COST  # Diagonal distance
        else:
            return NORMAL_MOVE_COST  # Orthogonal distance
    
    def set_movable_obstacles(self, obstacle_positions: dict):
        """Set movable obstacles from a dictionary of positions."""
        # First, clear all existing movable obstacles
        for row in self.nodes:
            for node in row:
                if node.node_type == NodeType.MOVABLE_OBSTACLE:
                    node.node_type = NodeType.EMPTY
        
        # Then, place the obstacles at their new positions
        for row, col in obstacle_positions:
            node = self.get_node(row, col)
            if node and node.node_type == NodeType.EMPTY:
                node.node_type = NodeType.MOVABLE_OBSTACLE