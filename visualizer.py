"""
Pygame-based visualization for A* pathfinding
"""
import pygame
import sys
from typing import Tuple, Optional
from grid import Grid, NodeType
from astar import AStarPathfinder

class PathfindingVisualizer:
    def __init__(self, grid_size: int = 20, cell_size: int = 35):
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.grid = Grid(grid_size)
        self.pathfinder = AStarPathfinder(self.grid)
        
        # Set initial algorithm configuration
        self.grid.neighbor_mode = 4
        self.grid.heuristic_mode = "manhattan"
        
        # Pygame setup
        pygame.init()
        self.width = grid_size * cell_size + 300  # Extra space for UI
        self.height = grid_size * cell_size + 150
        self.screen = pygame.display.set_mode((self.width, self.height))
        pygame.display.set_caption("A* Pathfinding Visualizer")
        
        # Colors with transparency support
        self.colors = {
            NodeType.EMPTY: (255, 255, 255),           # White
            NodeType.WALL: (50, 50, 50),               # Dark gray
            NodeType.MOVABLE_OBSTACLE: (200, 200, 200), # Light gray
            NodeType.START: (0, 200, 0),               # Green
            NodeType.GOAL: (200, 0, 0),                # Red
            NodeType.PATH: (255, 20, 147),             # Pink-purple path
            NodeType.OPEN: (255, 255, 0, 80),          # Yellow with low opacity
            NodeType.CLOSED: (0, 255, 0, 60),          # Green with low opacity
            NodeType.CURRENT: (255, 100, 255),         # Magenta
        }
        
        # UI colors
        self.ui_colors = {
            'background': (240, 240, 245),
            'button': (70, 130, 180),
            'button_hover': (100, 150, 200),
            'button_text': (255, 255, 255),
            'text': (50, 50, 50),
            'panel': (250, 250, 255),
            'border': (200, 200, 200),
            'dropdown': (255, 255, 255),
            'dropdown_border': (150, 150, 150),
        }
        
        # UI state
        self.dragging_start = False
        self.dragging_goal = False
        self.drawing_walls = False
        self.erasing_walls = False
        self.mouse_pressed = False
        self.last_wall_pos = None
        
        # Animation modes
        self.running_algorithm = False
        self.step_mode = False  # True for step-by-step, False for continuous
        self.algorithm_speed = 100  # milliseconds between steps
        self.last_step_time = 0
        
        # Algorithm configuration
        self.neighbor_mode = 4  # 4 or 8 neighbors
        self.heuristic_mode = "manhattan"  # "manhattan", "euclidean", "octile"
        
        # Obstacle placement mode
        self.obstacle_mode = "normal"  # "normal" for walls, "movable" for movable obstacles
        
        # Node placement modes
        self.placement_mode = "obstacle"  # "obstacle", "start", "goal"
        
        # Dropdown states
        self.neighbor_dropdown_open = False
        self.heuristic_dropdown_open = False
        
        # Font for UI
        self.font = pygame.font.Font(None, 20)
        self.button_font = pygame.font.Font(None, 24)
        
        # Button definitions
        button_width = 120
        button_height = 35
        button_spacing = 10
        start_x = self.grid_size * self.cell_size + 20
        start_y = 20
        
        self.buttons = {
            'run': pygame.Rect(start_x, start_y, button_width, button_height),
            'step': pygame.Rect(start_x, start_y + button_height + button_spacing, button_width, button_height),
            'reset': pygame.Rect(start_x, start_y + 2 * (button_height + button_spacing), button_width, button_height),
            'clear': pygame.Rect(start_x, start_y + 3 * (button_height + button_spacing), button_width, button_height),
        }
        
        # Dropdown definitions
        dropdown_y = start_y + 4 * (button_height + button_spacing) + 20
        dropdown_width = 100
        dropdown_height = 30
        
        self.dropdowns = {
            'neighbors': {
                'rect': pygame.Rect(start_x, dropdown_y, dropdown_width, dropdown_height),
                'options': ['4-way', '8-way'],
                'selected': 0,  # Default to 4-way
                'open': False
            },
            'heuristic': {
                'rect': pygame.Rect(start_x, dropdown_y + dropdown_height + 40, dropdown_width, dropdown_height),
                'options': ['Manhattan', 'Euclidean', 'Octile'],
                'selected': 0,  # Default to Manhattan
                'open': False
            }
        }
        
        self.button_hover = None
        
        # Set pathfinder callback
        self.pathfinder.set_visualization_callback(self.update_display)
        
    def get_grid_pos(self, mouse_pos: Tuple[int, int]) -> Optional[Tuple[int, int]]:
        """Convert mouse position to grid coordinates"""
        x, y = mouse_pos
        if 0 <= x < self.grid_size * self.cell_size and 0 <= y < self.grid_size * self.cell_size:
            return (y // self.cell_size, x // self.cell_size)
        return None
        
    def draw_grid(self):
        """Draw the grid and all nodes"""
        for row in range(self.grid_size):
            for col in range(self.grid_size):
                node = self.grid.get_node(row, col)
                
                rect = pygame.Rect(
                    col * self.cell_size,
                    row * self.cell_size,
                    self.cell_size,
                    self.cell_size
                )
                
                # Draw base color
                if node.node_type in [NodeType.EMPTY, NodeType.START, NodeType.GOAL]:
                    color = self.colors[NodeType.EMPTY]
                elif node.node_type == NodeType.WALL:
                    color = self.colors[NodeType.WALL]
                elif node.node_type == NodeType.MOVABLE_OBSTACLE:
                    color = self.colors[NodeType.MOVABLE_OBSTACLE]
                else:
                    color = self.colors[NodeType.EMPTY]
                
                pygame.draw.rect(self.screen, color, rect)
                
                # Draw '#' symbol for movable obstacles
                if node.node_type == NodeType.MOVABLE_OBSTACLE:
                    text_surface = self.font.render('#', True, (80, 80, 80))
                    text_rect = text_surface.get_rect(center=rect.center)
                    self.screen.blit(text_surface, text_rect)
                
                # Draw transparent overlays for open/closed sets
                if node.node_type in [NodeType.OPEN, NodeType.CLOSED]:
                    # Create a surface with per-pixel alpha
                    overlay = pygame.Surface((self.cell_size, self.cell_size), pygame.SRCALPHA)
                    overlay_color = self.colors[node.node_type]
                    overlay.fill(overlay_color)
                    self.screen.blit(overlay, (col * self.cell_size, row * self.cell_size))
                
                # Draw current node
                elif node.node_type == NodeType.CURRENT:
                    pygame.draw.rect(self.screen, self.colors[NodeType.CURRENT], rect)
                
                # Draw path
                elif node.node_type == NodeType.PATH:
                    pygame.draw.rect(self.screen, self.colors[NodeType.PATH], rect)
                
                # Draw grid lines
                pygame.draw.rect(self.screen, self.ui_colors['border'], rect, 1)
                
        # Draw start and goal as circles after everything else
        self.draw_start_goal_markers()
        
        # Draw path line overlay
        self.draw_path_line()
                
    def draw_start_goal_markers(self):
        """Draw start and goal as circles/dots"""
        if self.grid.start_pos:
            row, col = self.grid.start_pos
            center_x = col * self.cell_size + self.cell_size // 2
            center_y = row * self.cell_size + self.cell_size // 2
            radius = min(self.cell_size // 3, 12)
            
            # Draw white outline
            pygame.draw.circle(self.screen, (255, 255, 255), (center_x, center_y), radius + 2)
            # Draw green start marker
            pygame.draw.circle(self.screen, self.colors[NodeType.START], (center_x, center_y), radius)
            
        if self.grid.goal_pos:
            row, col = self.grid.goal_pos
            center_x = col * self.cell_size + self.cell_size // 2
            center_y = row * self.cell_size + self.cell_size // 2
            radius = min(self.cell_size // 3, 12)
            
            # Draw white outline
            pygame.draw.circle(self.screen, (255, 255, 255), (center_x, center_y), radius + 2)
            # Draw red goal marker
            pygame.draw.circle(self.screen, self.colors[NodeType.GOAL], (center_x, center_y), radius)
            
    def draw_path_line(self):
        """Draw pink-purple line connecting path nodes"""
        if self.pathfinder.path and len(self.pathfinder.path) > 1:
            path_points = []
            for node in self.pathfinder.path:
                center_x = node.col * self.cell_size + self.cell_size // 2
                center_y = node.row * self.cell_size + self.cell_size // 2
                path_points.append((center_x, center_y))
            
            # Draw thick pink-purple line
            if len(path_points) > 1:
                pygame.draw.lines(self.screen, self.colors[NodeType.PATH], False, path_points, 4)
                
    def draw_buttons(self):
        """Draw modern UI buttons"""
        # Check mouse position for hover effects
        mouse_pos = pygame.mouse.get_pos()
        
        for button_name, button_rect in self.buttons.items():
            # Determine button state
            is_hover = button_rect.collidepoint(mouse_pos)
            is_disabled = False
            
            # Button state logic
            if button_name == 'run':
                is_disabled = self.running_algorithm or not (self.grid.start_pos and self.grid.goal_pos)
                button_text = "Stop" if self.running_algorithm else "Run"
            elif button_name == 'step':
                is_disabled = self.running_algorithm or not (self.grid.start_pos and self.grid.goal_pos)
                button_text = "Step"
            elif button_name == 'reset':
                button_text = "Reset"
            elif button_name == 'clear':
                is_disabled = self.running_algorithm
                button_text = "Clear Path"
            
            # Choose button color
            if is_disabled:
                button_color = (150, 150, 150)
                text_color = (200, 200, 200)
            elif is_hover:
                button_color = self.ui_colors['button_hover']
                text_color = self.ui_colors['button_text']
            else:
                button_color = self.ui_colors['button']
                text_color = self.ui_colors['button_text']
            
            # Draw button with rounded corners effect
            pygame.draw.rect(self.screen, button_color, button_rect)
            pygame.draw.rect(self.screen, self.ui_colors['border'], button_rect, 2)
            
            # Draw button text
            text_surface = self.button_font.render(button_text, True, text_color)
            text_rect = text_surface.get_rect(center=button_rect.center)
            self.screen.blit(text_surface, text_rect)
            
    def draw_dropdowns(self):
        """Draw dropdown menus"""
        # Draw labels first (so they appear behind dropdown options)
        label_y_neighbors = self.dropdowns['neighbors']['rect'].y - 25
        label_y_heuristic = self.dropdowns['heuristic']['rect'].y - 25
        
        neighbors_label = self.font.render("Neighbors:", True, self.ui_colors['text'])
        heuristic_label = self.font.render("Heuristic:", True, self.ui_colors['text'])
        
        self.screen.blit(neighbors_label, (self.dropdowns['neighbors']['rect'].x, label_y_neighbors))
        self.screen.blit(heuristic_label, (self.dropdowns['heuristic']['rect'].x, label_y_heuristic))
        
        # Draw closed dropdowns first
        for dropdown_name, dropdown in self.dropdowns.items():
            if not dropdown['open']:
                self.draw_single_dropdown(dropdown_name, dropdown)
        
        # Draw open dropdowns last (on top of everything including labels)
        for dropdown_name, dropdown in self.dropdowns.items():
            if dropdown['open']:
                self.draw_single_dropdown(dropdown_name, dropdown)
            
    def draw_single_dropdown(self, name: str, dropdown: dict):
        """Draw a single dropdown menu"""
        rect = dropdown['rect']
        options = dropdown.get('options', [])
        selected = dropdown.get('selected', 0)
        is_open = dropdown.get('open', False)
        
        if not options:
            return
            
        # Ensure selected index is valid
        if selected >= len(options):
            selected = 0
            dropdown['selected'] = 0
        
        # Draw main dropdown box
        pygame.draw.rect(self.screen, self.ui_colors['dropdown'], rect)
        pygame.draw.rect(self.screen, self.ui_colors['dropdown_border'], rect, 2)
        
        # Draw selected option text
        selected_text = options[selected]
        text_surface = self.font.render(selected_text, True, self.ui_colors['text'])
        text_rect = text_surface.get_rect(center=(rect.centerx - 10, rect.centery))
        self.screen.blit(text_surface, text_rect)
        
        # Draw dropdown arrow
        arrow_x = rect.right - 15
        arrow_y = rect.centery
        if is_open:
            # Up arrow
            pygame.draw.polygon(self.screen, self.ui_colors['text'], [
                (arrow_x, arrow_y + 3), (arrow_x + 5, arrow_y - 3), (arrow_x + 10, arrow_y + 3)
            ])
        else:
            # Down arrow
            pygame.draw.polygon(self.screen, self.ui_colors['text'], [
                (arrow_x, arrow_y - 3), (arrow_x + 5, arrow_y + 3), (arrow_x + 10, arrow_y - 3)
            ])
        
        # Draw dropdown options if open
        if is_open:
            # Draw background panel for all options with subtle shadow
            total_height = len(options) * rect.height
            options_bg_rect = pygame.Rect(rect.x, rect.bottom, rect.width, total_height)
            
            # Draw subtle soft shadow with multiple layers for smoothness
            shadow_offset = 2
            for i in range(3):
                shadow_alpha = 30 - (i * 8)  # Gradually fade the shadow
                shadow_rect = pygame.Rect(
                    rect.x + shadow_offset + i, 
                    rect.bottom + shadow_offset + i, 
                    rect.width, 
                    total_height
                )
                shadow_surface = pygame.Surface((rect.width, total_height), pygame.SRCALPHA)
                shadow_surface.fill((0, 0, 0, shadow_alpha))
                self.screen.blit(shadow_surface, (shadow_rect.x, shadow_rect.y))
            
            # Create a completely opaque surface for the dropdown
            dropdown_surface = pygame.Surface((rect.width, total_height), pygame.SRCALPHA, 32)
            dropdown_surface = dropdown_surface.convert_alpha()
            
            # Fill the entire surface with solid white (255 alpha = fully opaque)
            dropdown_surface.fill((255, 255, 255, 255))
            
            # Blit the solid surface to the screen
            self.screen.blit(dropdown_surface, (rect.x, rect.bottom))
            
            # Draw subtle border around entire options area
            pygame.draw.rect(self.screen, (180, 180, 180), options_bg_rect, 2)
            
            mouse_pos = pygame.mouse.get_pos()
            for i, option in enumerate(options):
                option_rect = pygame.Rect(
                    rect.x, 
                    rect.bottom + i * rect.height, 
                    rect.width, 
                    rect.height
                )
                
                # Draw hover effect with another opaque surface
                if option_rect.collidepoint(mouse_pos):
                    hover_surface = pygame.Surface((rect.width, rect.height), pygame.SRCALPHA, 32)
                    hover_surface = hover_surface.convert_alpha()
                    hover_surface.fill((230, 240, 255, 255))  # Light blue, fully opaque
                    self.screen.blit(hover_surface, (rect.x, rect.bottom + i * rect.height))
                
                # Draw separator line (except for last item)
                if i < len(options) - 1:
                    line_y = option_rect.bottom
                    pygame.draw.line(self.screen, (200, 200, 200), 
                                   (option_rect.left + 8, line_y), 
                                   (option_rect.right - 8, line_y), 1)
                
                # Draw option text with anti-aliasing
                option_text = self.font.render(option, True, (40, 40, 40))
                option_text_rect = option_text.get_rect(center=option_rect.center)
                self.screen.blit(option_text, option_text_rect)
        
        # Instructions
    def draw_ui(self):
        """Draw UI text and information"""
        ui_x = self.grid_size * self.cell_size + 20
        y_offset = 350  # Start below buttons and dropdowns
        
        # Instructions  
        obstacle_type = "movable obstacles (#)" if self.obstacle_mode == "movable" else "walls"
        placement_text = {
            "obstacle": f"Left click + drag: Add/remove {obstacle_type}",
            "start": "Click to place start position (◯)",
            "goal": "Click to place goal position (☆)"
        }
        
        instructions = [
            "Controls:",
            placement_text[self.placement_mode],
            "S: Start placement mode" if self.placement_mode != "start" else ">>> START PLACEMENT MODE <<<",
            "G: Goal placement mode" if self.placement_mode != "goal" else ">>> GOAL PLACEMENT MODE <<<", 
            "O: Toggle obstacle type",
            "SPACE: Find path",
            "ENTER: Step through execution" if hasattr(self.pathfinder, 'world_path') and self.pathfinder.world_path else "",
            "",
            f"Execution: {'Running' if self.running_algorithm else 'Stopped'}",
            f"Mode: {'Step-by-step' if self.step_mode else 'Continuous'}",
            f"Speed: {self.algorithm_speed}ms",
            f"Obstacle mode: {self.obstacle_mode}",
            "",
            "Push mechanics: enabled" if hasattr(self.pathfinder, 'push_enabled') and self.pathfinder.push_enabled else "",
        ]
        
        for instruction in instructions:
            text = self.font.render(instruction, True, self.ui_colors['text'])
            self.screen.blit(text, (ui_x, y_offset))
            y_offset += 22
            
        # Queue information
        if hasattr(self.pathfinder, 'current_world_state') and self.pathfinder.current_world_state:
            y_offset += 10
            world_info = [
                "Push Search Status:",
                f"Open states: {len(self.pathfinder.world_open_set) if hasattr(self.pathfinder, 'world_open_set') else 0}",
                f"Closed states: {len(self.pathfinder.world_closed_set) if hasattr(self.pathfinder, 'world_closed_set') else 0}",
                f"Current cost: {self.pathfinder.current_world_state.g_cost:.1f}",
            ]
            
            for info in world_info:
                text = self.font.render(info, True, self.ui_colors['text'])
                self.screen.blit(text, (ui_x, y_offset))
                y_offset += 22
                
        elif hasattr(self.pathfinder, 'open_set') and self.pathfinder.open_set:
            y_offset += 10
            queue_info = [
                "Algorithm Status:",
                f"Open nodes: {len(self.pathfinder.open_set)}",
                f"Closed nodes: {len(self.pathfinder.closed_set)}",
            ]
            
            for info in queue_info:
                text = self.font.render(info, True, self.ui_colors['text'])
                self.screen.blit(text, (ui_x, y_offset))
                y_offset += 22
                
        # Current node info
        current = self.pathfinder.get_current_node()
        if current:
            y_offset += 10
            current_info = [
                f"Current: ({current.row}, {current.col})",
                f"G-cost: {current.g_cost:.1f}",
                f"H-cost: {current.h_cost:.1f}",
                f"F-cost: {current.f_cost:.1f}",
            ]
            
            for info in current_info:
                text = self.font.render(info, True, self.ui_colors['text'])
                self.screen.blit(text, (ui_x, y_offset))
                y_offset += 22
                
    def update_display(self):
        """Update the display (called by pathfinder)"""
        self.screen.fill(self.ui_colors['background'])  # Modern background
        self.draw_grid()
        self.draw_buttons()
        self.draw_ui()
        # Draw dropdowns LAST to ensure they're on top
        self.draw_dropdowns()
        pygame.display.flip()
        
    def handle_mouse_click(self, pos: Tuple[int, int], button: int):
        """Handle mouse click events"""
        # Check if click is on a button first
        if button == 1:  # Left click
            # Check dropdown clicks first
            if self.handle_dropdown_click(pos):
                return
                
            # Check button clicks
            for button_name, button_rect in self.buttons.items():
                if button_rect.collidepoint(pos):
                    self.handle_button_click(button_name)
                    return
        
        # Handle grid clicks
        grid_pos = self.get_grid_pos(pos)
        if not grid_pos:
            # Close all dropdowns if clicking outside everything
            if button == 1:
                for dropdown in self.dropdowns.values():
                    dropdown['open'] = False
            return
            
        row, col = grid_pos
        
        if button == 1:  # Left click
            if not self.running_algorithm:
                node = self.grid.get_node(row, col)
                if node:
                    if self.placement_mode == "start":
                        # Place start and return to obstacle mode
                        self.grid.set_start(row, col)
                        self.pathfinder.reset()
                        self.placement_mode = "obstacle"
                        print("Start placed! Returned to obstacle mode.")
                        
                    elif self.placement_mode == "goal":
                        # Place goal and return to obstacle mode  
                        self.grid.set_goal(row, col)
                        self.pathfinder.reset()
                        self.placement_mode = "obstacle"
                        print("Goal placed! Returned to obstacle mode.")
                        
                    elif self.placement_mode == "obstacle":
                        # Place/remove obstacles (existing logic)
                        if node.node_type not in [NodeType.START, NodeType.GOAL]:
                            # Determine what type of obstacle to place/remove
                            target_obstacle_type = NodeType.MOVABLE_OBSTACLE if self.obstacle_mode == "movable" else NodeType.WALL
                            
                            if node.node_type == target_obstacle_type:
                                # Remove existing obstacle of this type
                                self.erasing_walls = True
                                self.drawing_walls = False
                                node.node_type = NodeType.EMPTY
                            elif node.node_type in [NodeType.WALL, NodeType.MOVABLE_OBSTACLE]:
                                # Replace with current obstacle type
                                node.node_type = target_obstacle_type
                                self.drawing_walls = True
                                self.erasing_walls = False
                            else:
                                # Place new obstacle
                                self.drawing_walls = True
                                self.erasing_walls = False
                                node.node_type = target_obstacle_type
                            self.last_wall_pos = (row, col)
                
    def handle_dropdown_click(self, pos: Tuple[int, int]) -> bool:
        """Handle dropdown click events. Returns True if a dropdown was clicked."""
        for dropdown_name, dropdown in self.dropdowns.items():
            main_rect = dropdown['rect']
            
            # Click on main dropdown
            if main_rect.collidepoint(pos):
                # Close other dropdowns
                for other_name, other_dropdown in self.dropdowns.items():
                    if other_name != dropdown_name:
                        other_dropdown['open'] = False
                # Toggle this dropdown
                dropdown['open'] = not dropdown['open']
                return True
                
            # Click on dropdown options (only if this dropdown is open)
            if dropdown.get('open', False):
                options = dropdown.get('options', [])
                for i, option in enumerate(options):
                    option_rect = pygame.Rect(
                        main_rect.x,
                        main_rect.bottom + i * main_rect.height,
                        main_rect.width,
                        main_rect.height
                    )
                    if option_rect.collidepoint(pos):
                        dropdown['selected'] = i
                        dropdown['open'] = False
                        self.update_algorithm_settings(dropdown_name, i)
                        return True
        
        # If we get here, no dropdown was clicked, so close all
        for dropdown in self.dropdowns.values():
            dropdown['open'] = False
        return False
        
    def update_algorithm_settings(self, dropdown_name: str, selected_index: int):
        """Update algorithm settings based on dropdown selection"""
        if dropdown_name == 'neighbors':
            self.neighbor_mode = 4 if selected_index == 0 else 8
            self.grid.neighbor_mode = self.neighbor_mode
        elif dropdown_name == 'heuristic':
            heuristics = ['manhattan', 'euclidean', 'octile']
            self.heuristic_mode = heuristics[selected_index]
            self.grid.heuristic_mode = self.heuristic_mode
        
        # Reset pathfinder if settings changed
        if not self.running_algorithm:
            self.pathfinder.reset()
                
    def handle_button_click(self, button_name: str):
        """Handle button click events"""
        if button_name == 'run':
            if self.running_algorithm:
                self.stop_algorithm()
            else:
                self.start_algorithm(continuous=True)
        elif button_name == 'step':
            if not self.running_algorithm:
                self.single_step()
        elif button_name == 'reset':
            self.reset_grid()
        elif button_name == 'clear':
            if not self.running_algorithm:
                self.pathfinder.reset(restore_obstacle=True)
                
    def handle_mouse_drag(self, pos: Tuple[int, int]):
        """Handle mouse drag events for drawing walls"""
        # Only allow dragging in obstacle placement mode
        if self.placement_mode != "obstacle":
            return
            
        grid_pos = self.get_grid_pos(pos)
        if not grid_pos or self.running_algorithm:
            return
            
        row, col = grid_pos
        
        # Avoid redrawing the same cell repeatedly
        if self.last_wall_pos == (row, col):
            return
            
        node = self.grid.get_node(row, col)
        if not node or node.node_type in [NodeType.START, NodeType.GOAL]:
            return
            
        target_obstacle_type = NodeType.MOVABLE_OBSTACLE if self.obstacle_mode == "movable" else NodeType.WALL
        
        if self.drawing_walls and node.node_type not in [NodeType.WALL, NodeType.MOVABLE_OBSTACLE]:
            node.node_type = target_obstacle_type
            self.last_wall_pos = (row, col)
        elif self.erasing_walls and node.node_type in [NodeType.WALL, NodeType.MOVABLE_OBSTACLE]:
            node.node_type = NodeType.EMPTY
            self.last_wall_pos = (row, col)
                
    def handle_key_press(self, key):
        """Handle keyboard events"""
        if key == pygame.K_SPACE:
            if not self.running_algorithm:
                self.start_algorithm(continuous=True)
            else:
                self.stop_algorithm()
                
        elif key == pygame.K_s:
            if not self.running_algorithm:
                # S key - set start placement mode
                self.placement_mode = "start"
                print("Start placement mode - click to place start position (◯)")
                
        elif key == pygame.K_g:
            if not self.running_algorithm:
                # G key - set goal placement mode
                self.placement_mode = "goal" 
                print("Goal placement mode - click to place goal position (☆)")
                
        elif key == pygame.K_RETURN or key == pygame.K_KP_ENTER:
            self.single_step()
                
        elif key == pygame.K_r:
            self.reset_grid()
            
        elif key == pygame.K_c:
            if not self.running_algorithm:
                self.pathfinder.reset(restore_obstacle=True)
                
        elif key == pygame.K_PLUS or key == pygame.K_EQUALS:
            self.algorithm_speed = max(10, self.algorithm_speed - 20)
            
        elif key == pygame.K_MINUS:
            self.algorithm_speed = min(1000, self.algorithm_speed + 20)
            
        elif key == pygame.K_o:
            # Toggle obstacle mode between normal walls and movable obstacles
            if self.obstacle_mode == "normal":
                self.obstacle_mode = "movable"
                print("Switched to movable obstacle mode - click to place pushable obstacles (#)")
            else:
                self.obstacle_mode = "normal" 
                print("Switched to normal obstacle mode - click to place walls")
            
    def start_algorithm(self, continuous: bool = True):
        """Start the pathfinding algorithm"""
        if self.grid.start_pos and self.grid.goal_pos:
            # If a path is already found and we are not in continuous mode, just execute the next step
            if not continuous and hasattr(self.pathfinder, 'world_path') and self.pathfinder.world_path:
                self.pathfinder.execute_next_world_step()
                return

            self.pathfinder.reset(restore_obstacle=False)
            
            # Check if we need push mechanics
            movable_obstacles = self.pathfinder.get_movable_obstacles()
            
            if self.pathfinder.push_enabled and movable_obstacles:
                print("Planning with push mechanics (background)...")
                # Plan completely in background for push mechanics
                path = self.pathfinder.find_path(step_by_step=False)
                if path:
                    print("Path found! Use ENTER to step through execution or SPACE again for continuous execution.")
                    if continuous:
                        # Auto-start world path execution
                        self.step_mode = False
                        self.running_algorithm = True
                        self.last_step_time = pygame.time.get_ticks()
                    else:
                        # Set up for manual stepping through world path by executing the first step
                        self.step_mode = True
                        self.running_algorithm = False
                        self.pathfinder.reset_world_execution()
                        self.pathfinder.execute_next_world_step()
                else:
                    print("No path found!")
            else:
                # Traditional pathfinding - always run in background now
                print("Planning path (background)...")
                path = self.pathfinder.find_path(step_by_step=False)
                if path:
                    print("Path found!")
                    # Don't auto-start execution for traditional pathfinding
                    self.running_algorithm = False
                    self.step_mode = False
                else:
                    print("No path found!")
                
    def single_step(self):
        """Perform a single step of the algorithm or world path execution"""
        self.start_algorithm(continuous=False)
        
    def stop_algorithm(self):
        """Stop the pathfinding algorithm"""
        self.running_algorithm = False
        self.step_mode = False
        
    def reset_grid(self):
        """Reset the entire grid"""
        self.stop_algorithm()
        self.grid = Grid(self.grid_size)
        self.pathfinder = AStarPathfinder(self.grid)
        self.pathfinder.set_visualization_callback(self.update_display)
        
        # Reset mouse state
        self.mouse_pressed = False
        self.drawing_walls = False
        self.erasing_walls = False
        self.last_wall_pos = None
        self.step_mode = False
        
    def run_algorithm_step(self):
        """Run one step of the algorithm"""
        if self.running_algorithm:
            current_time = pygame.time.get_ticks()
            if current_time - self.last_step_time >= self.algorithm_speed:
                
                # Check if we're executing a world path
                if (hasattr(self.pathfinder, 'world_path') and 
                    self.pathfinder.world_path and 
                    hasattr(self.pathfinder, 'world_path_index')):
                    # Execute next step of world path
                    has_more_steps = self.pathfinder.execute_next_world_step()
                    if not has_more_steps:
                        print("World path execution completed!")
                        self.running_algorithm = False
                else:
                    # Regular algorithm step
                    result = self.pathfinder.step()
                    if result is True:  # Path found
                        self.running_algorithm = False
                    elif result is None:  # No path possible
                        self.running_algorithm = False
                
                self.last_step_time = current_time
                    
    def run(self):
        """Main game loop"""
        clock = pygame.time.Clock()
        running = True
        
        # Set initial start and goal
        self.grid.set_start(1, 1)
        self.grid.set_goal(self.grid_size - 2, self.grid_size - 2)
        
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                    
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    self.mouse_pressed = True
                    self.handle_mouse_click(event.pos, event.button)
                    
                elif event.type == pygame.MOUSEBUTTONUP:
                    self.mouse_pressed = False
                    self.drawing_walls = False
                    self.erasing_walls = False
                    self.last_wall_pos = None
                    
                elif event.type == pygame.MOUSEMOTION:
                    if self.mouse_pressed and (self.drawing_walls or self.erasing_walls):
                        self.handle_mouse_drag(event.pos)
                    
                elif event.type == pygame.KEYDOWN:
                    self.handle_key_press(event.key)
                    
            # Run algorithm step if running
            self.run_algorithm_step()
            
            # Update display
            self.update_display()
            
            # Control frame rate
            clock.tick(60)
            
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    visualizer = PathfindingVisualizer(grid_size=25, cell_size=30)
    visualizer.run()