import numpy as np
from scipy.spatial import KDTree
import random


class Node:
    def __init__(self, position):
        self.position = np.array(position)
        self.parent = None
        self.cost = 0.0


class RRTPlanner:
    def __init__(self, start, goal, grid_size, goal_radius=0.2, occupancy_grid=None,
                 origin=np.array([0.0, 0.0]), resolution=0.05,
                 step_size=0.2, max_nodes=10000, tree_type="RRT*"):

        self.start = Node(start)
        self.goal = np.array(goal)
        self.goal_radius = goal_radius
        self.step_size = step_size
        self.max_nodes = max_nodes
        self.tree_type = tree_type
        self.grid_size = grid_size

        self.nodes = [self.start]
        self.kdtree = KDTree([self.start.position])
        self.kdtree_needs_update = False

        self.path = []
        self.goal_reached = False
        self.explore_bias = 0.05

        self.occupancy_grid = occupancy_grid
        self.origin = np.array(origin)
        self.resolution = resolution

    def add_node(self):
        if len(self.nodes) >= self.max_nodes:
            return

        rnd_point = self.sample(self.grid_size)
        nearest_node = self.get_nearest(rnd_point)
        direction = rnd_point - nearest_node.position
        length = np.linalg.norm(direction)
        if length == 0:
            return

        new_pos = nearest_node.position + self.step_size * direction / length
        if self.in_obstacle(new_pos):
            return

        new_node = Node(new_pos)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + np.linalg.norm(new_node.position - nearest_node.position)

        if self.tree_type == "RRT*":
            radius = max(2.0, self.step_size * 4)
            neighbors = self.get_neighbors(new_node.position, radius=radius)

            # Find better parent for new_node
            for neighbor in neighbors:
                new_cost = neighbor.cost + np.linalg.norm(new_node.position - neighbor.position)
                if new_cost < new_node.cost and self.is_collision_free(neighbor.position, new_node.position):
                    new_node.parent = neighbor
                    new_node.cost = new_cost

            self.nodes.append(new_node)
            self.kdtree_needs_update = True

            # Rewire neighbors to new_node if cheaper
            for neighbor in neighbors:
                potential_cost = new_node.cost + np.linalg.norm(neighbor.position - new_node.position)
                if potential_cost < neighbor.cost and self.is_collision_free(new_node.position, neighbor.position):
                    neighbor.parent = new_node
                    neighbor.cost = potential_cost

        else:
            self.nodes.append(new_node)
            self.kdtree_needs_update = True

    def update_start_or_goal(self, pos, is_start=False):
        if is_start:
            self.start.position = pos
            self.nodes = [self.start]
            self.kdtree = KDTree([self.start.position])
        else:
            self.goal = pos

    def sample(self, size):
        if self.goal_reached:
            return self.informed_sample()
        if random.random() < self.explore_bias:
            return self.goal
        return np.random.uniform(0, size, size=2)

    def informed_sample(self):
        c_best = self.path_length()
        c_min = np.linalg.norm(self.goal - self.start.position)
        if c_best == float('inf') or c_best <= c_min:
            return np.random.uniform(0, self.grid_size, size=2)

        center = (self.start.position + self.goal) / 2
        a1 = (self.goal - self.start.position) / c_min
        theta = np.arctan2(a1[1], a1[0])
        rot = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])

        r1 = c_best / 2
        r2 = np.sqrt(c_best ** 2 - c_min ** 2) / 2 if c_best > c_min else 0.1

        # Ensure r1 >= r2; if not, rotate 90 deg to align long axis
        if r2 > r1:
            r1, r2 = r2, r1
            theta += np.pi / 2
            rot = np.array([[np.cos(theta), -np.sin(theta)],
                            [np.sin(theta), np.cos(theta)]])

        sample = np.random.randn(2) * [r1, r2]
        return np.dot(rot, sample) + center

    def get_nearest(self, point):
        if self.kdtree_needs_update:
            self.kdtree = KDTree([n.position for n in self.nodes])
            self.kdtree_needs_update = False
        _, idx = self.kdtree.query(point)
        return self.nodes[idx]

    def get_neighbors(self, point, radius):
        if self.kdtree_needs_update:
            self.kdtree = KDTree([n.position for n in self.nodes])
            self.kdtree_needs_update = False
        idxs = self.kdtree.query_ball_point(point, radius)
        return [self.nodes[i] for i in idxs]

    def is_collision_free(self, p1, p2, step=0.2):
        direction = p2 - p1
        dist = np.linalg.norm(direction)
        if dist == 0:
            return True
        direction /= dist
        for i in range(int(dist / step)):
            point = p1 + direction * i * step
            if self.in_obstacle(point):
                return False
        return True

    def in_obstacle(self, point):
        if self.occupancy_grid is None:
            return False

        gx = int((point[0] - self.origin[0]) / self.resolution)
        gy = self.grid_size - int((point[1] - self.origin[1]) / self.resolution) - 1

        if 0 <= gx < self.occupancy_grid.shape[1] and 0 <= gy < self.occupancy_grid.shape[0]:
            value = self.occupancy_grid[gy, gx]
            return value == 100 or value == -1
        return True

    def check_goal(self):
        best_node = None
        best_cost = float('inf')

        for node in self.nodes:
            if np.linalg.norm(node.position - self.goal) <= self.goal_radius:
                if node.cost < best_cost:
                    best_cost = node.cost
                    best_node = node

        if best_node is not None and (not self.goal_reached or best_cost < self.path_length()):
            self.goal_reached = True
            self.construct_path(best_node)

    def construct_path(self, node):
        self.path = []
        while node is not None:
            self.path.append(node.position)
            node = node.parent
        self.path.reverse()

    def path_length(self):
        if not self.goal_reached or not self.path:
            return float('inf')
        return sum(np.linalg.norm(self.path[i] - self.path[i - 1]) for i in range(1, len(self.path)))
