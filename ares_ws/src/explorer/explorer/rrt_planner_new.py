import numpy as np
from scipy.spatial import KDTree
from collections import deque
import random
import time
from typing import List, Optional, Tuple, Set


class Node:
    def __init__(self, position: np.ndarray, node_id: int = None):
        self.position = np.array(position, dtype=np.float64)
        self.parent: Optional['Node'] = None
        self.children: Set['Node'] = set()
        self.cost = 0.0
        self.id = node_id
        self.important = False  # flag for critical path nodes
        self.branch_value = 0.0  # value of this branch for pruning decisions
        self.last_accessed = time.time()  # for temporal pruning
        self.connection_count = 0  # number of times this node was part of a path
        self.is_start_node = False  # flag to identify start nodes

    def prune_nodes_near(self, pos, radius=0.3):
        """Remove nodes within radius of a given point (i.e., already traversed)"""
        keep_nodes = []
        for node in self.nodes:
            if np.linalg.norm(node.position - pos) > radius:
                keep_nodes.append(node)
        self.nodes = keep_nodes
        self.kdtree_needs_update = True

    def update_start_or_goal(self, pos, is_start=False):
        if is_start:
            new_start = Node(pos)
            self.start = new_start
            self.nodes.append(new_start)
            self.kdtree_needs_update = True
        else:
            self.goal = pos
        
    def add_child(self, child: 'Node'):
        self.children.add(child)
        child.parent = self
        
    def remove_child(self, child: 'Node'):
        if child in self.children:
            self.children.remove(child)
            child.parent = None
            
    def update_branch_value(self):
        """Calculate branch value based on connectivity and usage"""
        self.branch_value = (
            len(self.children) * 0.3 +  # connectivity
            self.connection_count * 0.5 +  # usage frequency
            (1.0 / (self.cost + 1e-6)) * 0.2  # inverse cost
        )


class InformedRRTStarFn2:
    def __init__(self, start: np.ndarray, goal: np.ndarray, grid_size: float,
                 goal_radius: float = 0.5, occupancy_grid: Optional[np.ndarray] = None,
                 origin: np.ndarray = np.array([0.0, 0.0]), resolution: float = 0.05,
                 step_size: float = 0.5, max_nodes: int = 2000, 
                 prune_threshold: float = 0.7, rewire_radius_multiplier: float = 3.0):
        
        # Core parameters
        self.grid_size = grid_size
        self.goal_radius = goal_radius
        self.step_size = step_size
        self.max_nodes = max_nodes
        self.prune_threshold = prune_threshold
        self.rewire_radius = rewire_radius_multiplier * step_size
        
        # Environment
        self.occupancy_grid = occupancy_grid
        self.origin = np.array(origin, dtype=np.float64)
        self.resolution = resolution
        
        # Tree structure
        self.start_node = Node(start, 0)
        self.start_node.is_start_node = True
        self.current_goal = np.array(goal, dtype=np.float64)
        self.nodes: List[Node] = [self.start_node]
        self.node_counter = 1
        self.kdtree: Optional[KDTree] = None
        self.kdtree_needs_update = True
        
        # Path tracking
        self.current_path: List[Node] = []
        self.goal_reached = False
        self.best_goal_node: Optional[Node] = None
        
        # Sampling parameters
        self.goal_bias = 0.1
        self.informed_sampling = False
        self.ellipse_params = None
        
        # Bot state
        self.current_position = np.array(start, dtype=np.float64)
        self.previous_goals: List[np.ndarray] = []
        self.movement_threshold = step_size * 0.5  # Threshold for updating start node
        
        # Performance tracking
        self.iteration_count = 0
        self.last_prune_time = time.time()
        self.prune_interval = 2.0  # seconds
        
        # Dynamic obstacle handling
        self.dynamic_obstacles: List[Tuple[np.ndarray, float, float]] = []  # (center, radius, timestamp)
        self.obstacle_memory_time = 10.0  # seconds
        
    def update_kdtree(self):
        """Update KDTree for efficient nearest neighbor queries"""
        if self.kdtree_needs_update and len(self.nodes) > 0:
            positions = [node.position for node in self.nodes]
            self.kdtree = KDTree(positions)
            self.kdtree_needs_update = False
    
    def get_nearest_node(self, point: np.ndarray) -> Node:
        """Find nearest node to given point"""
        self.update_kdtree()
        if len(self.nodes) == 1:
            return self.nodes[0]
        
        _, idx = self.kdtree.query(point)
        return self.nodes[idx]
    
    def get_neighbors(self, point: np.ndarray, radius: float) -> List[Node]:
        """Get all nodes within radius of point"""
        self.update_kdtree()
        if len(self.nodes) <= 1:
            return []
        
        indices = self.kdtree.query_ball_point(point, radius)
        return [self.nodes[i] for i in indices]
    
    def sample_point(self) -> np.ndarray:
        """Sample a point using informed sampling when possible"""
        self.iteration_count += 1
        
        # Goal biasing
        if random.random() < self.goal_bias:
            return self.current_goal.copy()
        
        # Informed sampling if path exists
        if self.informed_sampling and self.ellipse_params is not None:
            return self._sample_from_ellipse()
        
        # Multi-goal sampling - bias towards previous goal areas
        if len(self.previous_goals) > 0 and random.random() < 0.15:
            goal_idx = random.randint(0, len(self.previous_goals) - 1)
            noise = np.random.normal(0, self.step_size, 2)
            sample = self.previous_goals[goal_idx] + noise
            return np.clip(sample, 0, self.grid_size)
        
        # Uniform sampling
        return np.random.uniform(0, self.grid_size, 2)
    
    def _sample_from_ellipse(self) -> np.ndarray:
        """Sample from elliptical region for informed RRT*"""
        c_best, c_min, center, rotation_matrix, r1, r2 = self.ellipse_params
        
        # Sample from unit circle and transform
        angle = random.uniform(0, 2 * np.pi)
        radius = random.uniform(0, 1)
        
        # Transform to ellipse
        x = np.sqrt(radius) * np.cos(angle) * r1
        y = np.sqrt(radius) * np.sin(angle) * r2
        
        # Rotate and translate
        local_point = np.array([x, y])
        world_point = rotation_matrix @ local_point + center
        
        return np.clip(world_point, 0, self.grid_size)
    
    def steer(self, from_pos: np.ndarray, to_pos: np.ndarray) -> np.ndarray:
        """Steer from one position towards another with step size constraint"""
        direction = to_pos - from_pos
        distance = np.linalg.norm(direction)
        
        if distance <= self.step_size:
            return to_pos
        
        return from_pos + (direction / distance) * self.step_size
    
    def is_collision_free(self, p1: np.ndarray, p2: np.ndarray, 
                         check_dynamic: bool = True) -> bool:
        """Check if path between two points is collision-free"""
        # Static obstacle check
        if not self._check_static_collision_free(p1, p2):
            return False
        
        # Dynamic obstacle check
        if check_dynamic and not self._check_dynamic_collision_free(p1, p2):
            return False
        
        return True
    
    def _check_static_collision_free(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        """Check collision with static obstacles"""
        if self.occupancy_grid is None:
            return True
        
        # Line collision checking with higher resolution
        direction = p2 - p1
        distance = np.linalg.norm(direction)
        
        if distance == 0:
            return not self._point_in_obstacle(p1)
        
        # Check points along the line
        num_checks = max(int(distance / (self.resolution * 0.5)), 2)
        for i in range(num_checks + 1):
            t = i / num_checks
            point = p1 + t * direction
            if self._point_in_obstacle(point):
                return False
        return True
    
    def _check_dynamic_collision_free(self, p1: np.ndarray, p2: np.ndarray) -> bool:
        """Check collision with dynamic obstacles"""
        current_time = time.time()
        
        for obs_center, obs_radius, timestamp in self.dynamic_obstacles:
            if current_time - timestamp > self.obstacle_memory_time:
                continue
            
            # Check if line segment intersects with circular obstacle
            if self._line_circle_intersection(p1, p2, obs_center, obs_radius):
                return False
        
        return True
    
    def _line_circle_intersection(self, p1: np.ndarray, p2: np.ndarray, 
                                center: np.ndarray, radius: float) -> bool:
        """Check if line segment intersects with circle"""
        # Vector from p1 to p2
        d = p2 - p1
        # Vector from p1 to circle center
        f = p1 - center
        
        a = np.dot(d, d)
        b = 2 * np.dot(f, d)
        c = np.dot(f, f) - radius * radius
        
        discriminant = b * b - 4 * a * c
        
        if discriminant < 0:
            return False
        
        discriminant = np.sqrt(discriminant)
        t1 = (-b - discriminant) / (2 * a)
        t2 = (-b + discriminant) / (2 * a)
        
        return (0 <= t1 <= 1) or (0 <= t2 <= 1) or (t1 < 0 and t2 > 1)
    
    def _point_in_obstacle(self, point: np.ndarray) -> bool:
        if self.occupancy_grid is None:
            return False

        # Map world point to grid indices
        gx = int((point[0] - self.origin[0]) / self.resolution)
        gy = int((self.grid_size - point[1] - self.origin[1]) / self.resolution)
        # Check bounds
        if (0 <= gx < self.occupancy_grid.shape[1] and 
            0 <= gy < self.occupancy_grid.shape[0]):
            value = self.occupancy_grid[gy, gx]
            # Only 0 is free; 100 = wall, -1 = unknown
            return value == 100 or value == -1

        # Out of bounds - treat as obstacle
        return True
    
    def add_node_to_tree(self, new_position: np.ndarray, 
                        parent: Node) -> Optional[Node]:
        """Add a new node to the tree with RRT* optimization"""
        new_node = Node(new_position, self.node_counter)
        self.node_counter += 1
        
        # Initial parent assignment
        new_node.parent = parent
        new_node.cost = parent.cost + np.linalg.norm(new_position - parent.position)
        
        # Find neighbors for RRT* optimization  
        neighbors = self.get_neighbors(new_position, self.rewire_radius)
        
        # Choose best parent among neighbors
        best_parent = parent
        best_cost = new_node.cost
        
        for neighbor in neighbors:
            if neighbor == parent:
                continue
            
            potential_cost = (neighbor.cost + 
                            np.linalg.norm(new_position - neighbor.position))
            
            if (potential_cost < best_cost and 
                self.is_collision_free(neighbor.position, new_position)):
                best_parent = neighbor
                best_cost = potential_cost
        
        # Update parent if better one found
        if best_parent != parent:
            new_node.parent = best_parent
            new_node.cost = best_cost
        
        # Add to tree structure
        best_parent.add_child(new_node)
        self.nodes.append(new_node)
        self.kdtree_needs_update = True
        
        # Rewire neighbors if this provides better path
        for neighbor in neighbors:
            if neighbor == best_parent or neighbor == new_node:
                continue
            
            new_cost_via_new = (new_node.cost + 
                              np.linalg.norm(neighbor.position - new_position))
            
            if (new_cost_via_new < neighbor.cost and 
                self.is_collision_free(new_position, neighbor.position)):
                
                # Remove from old parent
                if neighbor.parent:
                    neighbor.parent.remove_child(neighbor)
                
                # Add to new parent
                new_node.add_child(neighbor)
                neighbor.cost = new_cost_via_new
                
                # Propagate cost changes to descendants
                self._propagate_cost_to_descendants(neighbor)
        
        return new_node
    
    def _propagate_cost_to_descendants(self, node: Node):
        """Propagate cost changes to all descendants"""
        queue = deque([node])
        
        while queue:
            current = queue.popleft()
            
            for child in current.children:
                old_cost = child.cost
                new_cost = (current.cost + 
                          np.linalg.norm(child.position - current.position))
                
                if new_cost < old_cost:
                    child.cost = new_cost
                    queue.append(child)
    
    def check_goal_connection(self) -> bool:
        """Check if any node can connect to goal and update best path"""
        best_node = None
        best_total_cost = float('inf')
        
        for node in self.nodes:
            distance_to_goal = np.linalg.norm(node.position - self.current_goal)
            
            if distance_to_goal <= self.goal_radius:
                total_cost = node.cost + distance_to_goal
                
                if (total_cost < best_total_cost and 
                    self.is_collision_free(node.position, self.current_goal)):
                    best_node = node
                    best_total_cost = total_cost
        
        if best_node is not None:
            if not self.goal_reached or best_total_cost < self._get_current_path_cost():
                self.best_goal_node = best_node
                self.goal_reached = True
                self._construct_path()
                self._update_informed_sampling()
                return True
        
        return False
    
    def _construct_path(self):
        """Construct path from start to goal through best goal node"""
        if self.best_goal_node is None:
            return
        
        self.current_path = []
        current = self.best_goal_node
        
        while current is not None:
            current.important = True
            current.connection_count += 1
            current.last_accessed = time.time()
            self.current_path.append(current)
            current = current.parent
        
        self.current_path.reverse()
    
    def _get_current_path_cost(self) -> float:
        """Get cost of current path"""
        if not self.current_path or not self.goal_reached:
            return float('inf')
        
        cost = self.best_goal_node.cost
        cost += np.linalg.norm(self.best_goal_node.position - self.current_goal)
        return cost
    
    def _update_informed_sampling(self):
        """Update informed sampling parameters"""
        if not self.goal_reached:
            return
        
        c_best = self._get_current_path_cost()
        c_min = np.linalg.norm(self.current_goal - self.start_node.position)
        
        if c_best == float('inf') or c_best <= c_min * 1.1:
            self.informed_sampling = False
            return
        
        # Calculate ellipse parameters
        center = (self.start_node.position + self.current_goal) / 2
        
        # Rotation matrix
        diff = self.current_goal - self.start_node.position
        angle = np.arctan2(diff[1], diff[0])
        cos_angle, sin_angle = np.cos(angle), np.sin(angle)
        rotation_matrix = np.array([[cos_angle, -sin_angle],
                                  [sin_angle, cos_angle]])
        
        # Ellipse radii
        r1 = c_best / 2
        r2 = np.sqrt(c_best**2 - c_min**2) / 2 if c_best > c_min else 0.1
        
        self.ellipse_params = (c_best, c_min, center, rotation_matrix, r1, r2)
        self.informed_sampling = True
    
    def update_goal(self, new_goal: np.ndarray):
        """Update goal and create new start node at bot's current position"""
        # Archive previous goal if significantly different
        if (len(self.previous_goals) == 0 or 
            np.linalg.norm(new_goal - self.current_goal) > self.goal_radius):
            self.previous_goals.append(self.current_goal.copy())
            # Limit previous goals memory
            if len(self.previous_goals) > 5:
                self.previous_goals.pop(0)

        # Update goal
        self.current_goal = np.array(new_goal, dtype=np.float64)
        
        # Reset goal-related state
        self.goal_reached = False
        self.best_goal_node = None
        self.current_path = []
        self.informed_sampling = False
        self.ellipse_params = None

        # Update goal bias based on distance
        distance_to_goal = np.linalg.norm(self.current_goal - self.current_position)
        self.goal_bias = min(0.3, 0.05 + 0.25 * np.exp(-distance_to_goal / 50.0))

        # Create new start node at current position
        self._create_new_start_node()
        
        print(f"New goal set: {self.current_goal}, new start at {self.start_node.position}")
    
    def _create_new_start_node(self):
        """Create a new start node at the bot's current position"""
        # Mark old start node as no longer the start
        if self.start_node:
            self.start_node.is_start_node = False
        
        # Create new start node
        new_start = Node(self.current_position.copy(), self.node_counter)
        new_start.is_start_node = True
        new_start.cost = 0.0
        self.node_counter += 1

        # Find the best connection point in the existing tree
        if len(self.nodes) > 0:
            # Get nearby nodes for potential connection
            nearby_nodes = self.get_neighbors(new_start.position, self.rewire_radius * 2)
            
            if not nearby_nodes:
                # If no nearby nodes, connect to the nearest node
                nearest = self.get_nearest_node(new_start.position)
                if self.is_collision_free(new_start.position, nearest.position):
                    nearby_nodes = [nearest]
            
            # Find the best connection that minimizes cost
            best_connection = None
            best_connection_cost = float('inf')
            
            for node in nearby_nodes:
                if self.is_collision_free(new_start.position, node.position):
                    connection_cost = np.linalg.norm(new_start.position - node.position)
                    if connection_cost < best_connection_cost:
                        best_connection = node
                        best_connection_cost = connection_cost
            
            # Connect to the best node
            if best_connection:
                new_start.add_child(best_connection)
                
                # Recalculate costs for the connected subtree
                best_connection.cost = best_connection_cost
                self._propagate_cost_to_descendants(best_connection)
                
                # Perform local rewiring to optimize connections
                self._local_rewire_around_new_start(new_start)

        # Update tree structure
        self.nodes.append(new_start)
        self.start_node = new_start
        self.kdtree_needs_update = True
    
    def _local_rewire_around_new_start(self, new_start: Node):
        """Perform local rewiring around the new start node"""
        neighbors = self.get_neighbors(new_start.position, self.rewire_radius * 1.5)
        
        for neighbor in neighbors:
            if neighbor == new_start or neighbor.parent == new_start:
                continue
            
            # Check if connecting through new_start improves the neighbor's cost
            new_cost_via_start = np.linalg.norm(new_start.position - neighbor.position)
            
            if (new_cost_via_start < neighbor.cost and 
                self.is_collision_free(new_start.position, neighbor.position)):
                
                # Remove from old parent
                if neighbor.parent:
                    neighbor.parent.remove_child(neighbor)
                
                # Connect to new start
                new_start.add_child(neighbor)
                neighbor.cost = new_cost_via_start
                
                # Propagate cost changes
                self._propagate_cost_to_descendants(neighbor)
    
    def update_current_position(self, new_position: np.ndarray):
        """Update bot's current position and potentially move start node"""
        old_position = self.current_position.copy()
        self.current_position = np.array(new_position, dtype=np.float64)
        
        # Check if we should update the start node position
        distance_moved = np.linalg.norm(self.current_position - self.start_node.position)
        
        if distance_moved > self.movement_threshold:
            self._update_start_node_position()
        
        # Check if we need to prune the tree
        current_time = time.time()
        if current_time - self.last_prune_time > self.prune_interval:
            self._intelligent_prune()
            self.last_prune_time = current_time
        
        # Update path validity if it exists
        if self.current_path:
            self._validate_current_path()
    
    def _update_start_node_position(self):
        """Update start node to current bot position with tree optimization"""
        old_start_pos = self.start_node.position.copy()
        
        # Update start node position
        self.start_node.position = self.current_position.copy()
        self.start_node.cost = 0.0
        
        # Recalculate costs for all children
        for child in self.start_node.children:
            child.cost = np.linalg.norm(self.start_node.position - child.position)
            self._propagate_cost_to_descendants(child)
        
        # Perform local optimization around new start position
        self._optimize_around_start()
        
        # Update KDTree since start position changed
        self.kdtree_needs_update = True
        
        # Reset informed sampling since start changed
        if self.informed_sampling:
            self._update_informed_sampling()
    
    def _optimize_around_start(self):
        """Optimize tree connections around the updated start position"""
        search_radius = self.rewire_radius * 2
        nearby_nodes = self.get_neighbors(self.start_node.position, search_radius)
        
        # Remove start node from neighbors list
        nearby_nodes = [n for n in nearby_nodes if n != self.start_node]
        
        for node in nearby_nodes:
            # Skip if already connected to start
            if node.parent == self.start_node:
                continue
            
            # Check if connecting to start improves cost
            direct_cost = np.linalg.norm(self.start_node.position - node.position)
            
            if (direct_cost < node.cost and 
                self.is_collision_free(self.start_node.position, node.position)):
                
                # Remove from old parent
                if node.parent:
                    node.parent.remove_child(node)
                
                # Connect to start
                self.start_node.add_child(node)
                node.cost = direct_cost
                
                # Propagate cost changes
                self._propagate_cost_to_descendants(node)
    
    def _validate_current_path(self):
        """Validate current path and invalidate if obstacles detected"""
        if not self.current_path:
            return
        
        # Check if path is still collision-free
        for i in range(len(self.current_path) - 1):
            if not self.is_collision_free(self.current_path[i].position, 
                                        self.current_path[i + 1].position):
                # Path is invalid, reset goal state
                self.goal_reached = False
                self.current_path = []
                self.best_goal_node = None
                print("Current path invalidated due to obstacles")
                break
    
    def add_dynamic_obstacle(self, center: np.ndarray, radius: float):
        """Add a dynamic obstacle with trajectory prediction"""
        self.dynamic_obstacles.append((center.copy(), radius, time.time()))
        
        # Remove old obstacles
        current_time = time.time()
        self.dynamic_obstacles = [
            (c, r, t) for c, r, t in self.dynamic_obstacles
            if current_time - t <= self.obstacle_memory_time
        ]
        
        # Invalidate affected paths and nodes
        self._handle_dynamic_obstacle(center, radius)
    
    def _handle_dynamic_obstacle(self, center: np.ndarray, radius: float):
        """Handle dynamic obstacle by invalidating affected paths and nodes"""
        # Invalidate current path if affected
        self._invalidate_paths_through_obstacle(center, radius)
        
        # Mark affected nodes for potential removal
        affected_nodes = []
        buffer_radius = radius * 1.2  # Add safety buffer
        
        for node in self.nodes:
            if node == self.start_node:  # Never remove start node
                continue
                
            distance_to_obstacle = np.linalg.norm(node.position - center)
            if distance_to_obstacle < buffer_radius:
                affected_nodes.append(node)
        
        # Remove affected nodes that aren't critical
        nodes_to_remove = set()
        for node in affected_nodes:
            if not node.important or node.connection_count < 2:
                nodes_to_remove.add(node)
        
        if nodes_to_remove:
            self._remove_nodes(nodes_to_remove)
            print(f"Removed {len(nodes_to_remove)} nodes due to dynamic obstacle")
    
    def _invalidate_paths_through_obstacle(self, center: np.ndarray, radius: float):
        """Invalidate paths that go through a dynamic obstacle"""
        if not self.current_path:
            return
        
        # Check if current path is affected
        path_affected = False
        for i in range(len(self.current_path) - 1):
            p1 = self.current_path[i].position
            p2 = self.current_path[i + 1].position
            
            if self._line_circle_intersection(p1, p2, center, radius):
                path_affected = True
                break
        
        if path_affected:
            self.goal_reached = False
            self.current_path = []
            self.best_goal_node = None
            print("Path invalidated due to dynamic obstacle")
    
    def _intelligent_prune(self):
        """Intelligent pruning to maintain tree quality while managing size"""
        if len(self.nodes) < self.max_nodes * 0.8:
            return
        
        # Calculate branch values for all nodes
        for node in self.nodes:
            node.update_branch_value()
        
        # Identify nodes to remove
        nodes_to_remove = set()
        current_time = time.time()
        
        for node in self.nodes:
            # Never remove start node
            if node.is_start_node:
                continue
            
            # Skip recently important nodes
            if node.important and current_time - node.last_accessed < 30.0:
                continue
            
            # Distance-based pruning
            distance_to_bot = np.linalg.norm(node.position - self.current_position)
            
            # Remove nodes that are very far and have low value
            if distance_to_bot > 150.0 and node.branch_value < 0.3:
                nodes_to_remove.add(node)
                continue
            
            # Remove nodes that haven't been accessed recently and have low value
            if (current_time - node.last_accessed > 90.0 and 
                node.branch_value < 0.8 and distance_to_bot > 75.0):
                nodes_to_remove.add(node)
        
        # Remove nodes and update tree structure
        if nodes_to_remove:
            self._remove_nodes(nodes_to_remove)
        
        # If still too many nodes, do aggressive pruning
        if len(self.nodes) > self.max_nodes:
            self._aggressive_prune()
    
    def _aggressive_prune(self):
        """More aggressive pruning when tree is still too large"""
        target_size = int(self.max_nodes * 0.6)
        nodes_to_keep = []
        
        # Always keep start node and highly important nodes
        for node in self.nodes:
            if (node.is_start_node or 
                (node.important and node.branch_value > 1.5) or 
                node.branch_value > 3.0):
                nodes_to_keep.append(node)
        
        # Sort remaining nodes by combined score
        remaining_nodes = [n for n in self.nodes if n not in nodes_to_keep]
        
        def pruning_score(node):
            distance_to_bot = np.linalg.norm(node.position - self.current_position)
            distance_to_goal = np.linalg.norm(node.position - self.current_goal)
            
            # Prefer nodes that are closer to bot/goal and have higher branch value
            return (-node.branch_value * 2.0 + 
                   distance_to_bot / 100.0 + 
                   distance_to_goal / 100.0)
        
        remaining_nodes.sort(key=pruning_score)
        
        # Keep best remaining nodes
        keep_count = max(0, target_size - len(nodes_to_keep))
        nodes_to_keep.extend(remaining_nodes[:keep_count])
        
        # Remove the rest
        nodes_to_remove = set(self.nodes) - set(nodes_to_keep)
        self._remove_nodes(nodes_to_remove)
    
    def _remove_nodes(self, nodes_to_remove: Set[Node]):
        """Remove nodes from tree and update structure"""
        if not nodes_to_remove:
            return
        
        # Remove from children lists and reparent orphaned nodes
        for node in nodes_to_remove:
            if node.parent and node in node.parent.children:
                node.parent.children.remove(node)
            
            # Reparent children to their grandparent or find new parent
            for child in node.children:
                if node.parent:
                    # Try to connect to grandparent
                    if self.is_collision_free(node.parent.position, child.position):
                        node.parent.add_child(child)
                        child.cost = (node.parent.cost + 
                                    np.linalg.norm(child.position - node.parent.position))
                        self._propagate_cost_to_descendants(child)
                    else:
                        # Find alternative parent
                        self._find_new_parent(child)
                else:
                    # Find any suitable parent
                    self._find_new_parent(child)
        
        # Remove from main nodes list
        self.nodes = [n for n in self.nodes if n not in nodes_to_remove]
        self.kdtree_needs_update = True
        
        print(f"Pruned {len(nodes_to_remove)} nodes. Tree size: {len(self.nodes)}")
    
    def _find_new_parent(self, orphaned_node: Node):
        """Find a new parent for an orphaned node"""
        # Get nearby nodes
        nearby_nodes = self.get_neighbors(orphaned_node.position, self.rewire_radius * 1.5)
        
        best_parent = None
        best_cost = float('inf')
        
        for candidate in nearby_nodes:
            if candidate == orphaned_node:
                continue
            
            potential_cost = (candidate.cost + 
                            np.linalg.norm(orphaned_node.position - candidate.position))
            
            if (potential_cost < best_cost and 
                self.is_collision_free(candidate.position, orphaned_node.position)):
                best_parent = candidate
                best_cost = potential_cost
        
        if best_parent:
            best_parent.add_child(orphaned_node)
            orphaned_node.cost = best_cost
            self._propagate_cost_to_descendants(orphaned_node)
        else:
            # If no parent found, remove this node too
            if orphaned_node in self.nodes:
                self.nodes.remove(orphaned_node)
    
    def plan_step(self) -> bool:
        """Execute one planning iteration"""
        # Sample new point
        sample_point = self.sample_point()
        
        # Find nearest node
        nearest_node = self.get_nearest_node(sample_point)
        
        # Steer towards sample
        new_position = self.steer(nearest_node.position, sample_point)
        
        # Check collision
        if not self.is_collision_free(nearest_node.position, new_position):
            return False
        
        # Add node to tree
        new_node = self.add_node_to_tree(new_position, nearest_node)
        
        if new_node is None:
            return False
        
        # Check goal connection
        goal_connected = self.check_goal_connection()
        
        return True
    
    def plan_multiple_steps(self, num_steps: int = 10) -> bool:
        """Execute multiple planning iterations for efficiency"""
        goal_found = False
        for _ in range(num_steps):
            if self.plan_step():
                if self.goal_reached:
                    goal_found = True
                    break
        return goal_found
    
    def get_path_to_goal(self) -> List[np.ndarray]:
        """Get current best path to goal as list of positions"""
        if not self.goal_reached or not self.current_path:
            return []
        
        path_positions = [node.position.copy() for node in self.current_path]
        # Add goal position
        path_positions.append(self.current_goal.copy())
        
        return path_positions
    
    def get_next_waypoint(self, lookahead_distance: float = None) -> Optional[np.ndarray]:
        """Get the next waypoint for the bot to follow"""
        if not self.current_path:
            return None
        
        if lookahead_distance is None:
            lookahead_distance = self.step_size * 2
        
        # Find the furthest point on path within lookahead distance
        current_pos = self.current_position
        
        for i, node in enumerate(self.current_path):
            distance = np.linalg.norm(node.position - current_pos)
            if distance <= self.goal_radius:
                # We've reached this node, remove it from path
                continue
            elif distance <= lookahead_distance:
                return node.position.copy()
            else:
                # This node is beyond lookahead, return it
                return node.position.copy()
        
        # If we've processed all path nodes, return goal
        return self.current_goal.copy()
    
    def get_tree_edges(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Get all tree edges for visualization"""
        edges = []
        for node in self.nodes:
            if node.parent is not None:
                edges.append((node.parent.position.copy(), node.position.copy()))
        return edges
    
    def get_important_edges(self) -> List[Tuple[np.ndarray, np.ndarray]]:
        """Get edges of important/high-value branches for visualization"""
        edges = []
        for node in self.nodes:
            if (node.parent is not None and 
                (node.important or node.branch_value > 1.5)):
                edges.append((node.parent.position.copy(), node.position.copy()))
        return edges
    
    def get_statistics(self) -> dict:
        """Get planner statistics"""
        important_nodes = sum(1 for n in self.nodes if n.important)
        avg_branch_value = np.mean([n.branch_value for n in self.nodes]) if self.nodes else 0
        
        return {
            'num_nodes': len(self.nodes),
            'num_important_nodes': important_nodes,
            'avg_branch_value': avg_branch_value,
            'goal_reached': self.goal_reached,
            'path_cost': self._get_current_path_cost() if self.goal_reached else float('inf'),
            'path_length': len(self.current_path) if self.current_path else 0,
            'iterations': self.iteration_count,
            'informed_sampling': self.informed_sampling,
            'num_previous_goals': len(self.previous_goals),
            'num_dynamic_obstacles': len(self.dynamic_obstacles),
            'start_position': self.start_node.position.copy(),
            'current_position': self.current_position.copy(),
            'goal_position': self.current_goal.copy()
        }
    
    def reset_tree(self):
        """Reset the entire tree (use sparingly)"""
        # Keep only start node
        self.start_node = Node(self.current_position.copy(), 0)
        self.start_node.is_start_node = True
        self.nodes = [self.start_node]
        self.node_counter = 1
        
        # Reset state
        self.current_path = []
        self.goal_reached = False
        self.best_goal_node = None
        self.informed_sampling = False
        self.ellipse_params = None
        self.kdtree_needs_update = True
        
        print("Tree reset to start node only")
    
    def get_path_smoothed(self, smoothing_factor: float = 0.3) -> List[np.ndarray]:
        """Get a smoothed version of the current path"""
        raw_path = self.get_path_to_goal()
        if len(raw_path) < 3:
            return raw_path
        
        smoothed_path = [raw_path[0]]  # Keep start
        
        for i in range(1, len(raw_path) - 1):
            # Apply simple smoothing
            prev_point = np.array(raw_path[i-1])
            curr_point = np.array(raw_path[i])
            next_point = np.array(raw_path[i+1])
            
            # Weighted average
            smoothed_point = (
                curr_point * (1 - smoothing_factor) + 
                (prev_point + next_point) * smoothing_factor / 2
            )
            
            # Check if smoothed path is collision-free
            if (self.is_collision_free(smoothed_path[-1], smoothed_point) and
                self.is_collision_free(smoothed_point, raw_path[i+1])):
                smoothed_path.append(smoothed_point)
            else:
                smoothed_path.append(curr_point)
        
        smoothed_path.append(raw_path[-1])  # Keep goal
        return smoothed_path
    
    def visualize_tree_info(self) -> dict:
        """Get information for tree visualization"""
        node_info = []
        for node in self.nodes:
            info = {
                'position': node.position.copy(),
                'is_start': node.is_start_node,
                'is_important': node.important,
                'branch_value': node.branch_value,
                'cost': node.cost,
                'num_children': len(node.children)
            }
            node_info.append(info)
        
        return {
            'nodes': node_info,
            'edges': self.get_tree_edges(),
            'important_edges': self.get_important_edges(),
            'current_path': self.get_path_to_goal(),
            'dynamic_obstacles': [(center.copy(), radius) for center, radius, _ in self.dynamic_obstacles]
        }