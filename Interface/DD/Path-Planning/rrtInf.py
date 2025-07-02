import numpy as np
from scipy.spatial import KDTree
import random

class Node:
    def __init__(self, position):
        self.position = np.array(position)
        self.parent = None
        self.cost = 0.0

class RRTPlanner:
    def __init__(self, start, goal, goal_radius=1.0, obstacles=None, step_size=1.0, max_nodes=10000, tree_type="RRT*"):
        self.start = Node(start)
        self.goal = np.array(goal)
        self.goal_radius = goal_radius
        self.obstacles = obstacles or []
        self.step_size = step_size
        self.max_nodes = max_nodes
        self.tree_type = tree_type

        self.nodes = [self.start]
        self.kdtree = KDTree([self.start.position])
        self.kdtree_needs_update = False

        self.path = []
        self.goal_reached = False
        self.explore_bias = 0.05  # 5% chance to sample goal

    def add_node(self):
        # if self.goal_reached or len(self.nodes) >= self.max_nodes:
        #     return

        rnd_point = self.sample()
        nearest_node = self.get_nearest(rnd_point)
        direction = rnd_point - nearest_node.position
        length = np.linalg.norm(direction)
        if length == 0:
            return

        new_pos = nearest_node.position + (self.step_size * direction / length)
        if self.in_obstacle(new_pos):
            return

        new_node = Node(new_pos)
        new_node.parent = nearest_node
        new_node.cost = nearest_node.cost + np.linalg.norm(new_node.position - nearest_node.position)

        if self.tree_type == "RRT*":
            neighbors = self.get_neighbors(new_node.position, radius=2.0)
            for neighbor in neighbors:
                new_cost = neighbor.cost + np.linalg.norm(new_node.position - neighbor.position)
                if new_cost < new_node.cost and self.is_collision_free(neighbor.position, new_node.position):
                    new_node.parent = neighbor
                    new_node.cost = new_cost

            # Rewire
            for neighbor in neighbors:
                potential_cost = new_node.cost + np.linalg.norm(neighbor.position - new_node.position)
                if potential_cost < neighbor.cost and self.is_collision_free(new_node.position, neighbor.position):
                    neighbor.parent = new_node
                    neighbor.cost = potential_cost

        self.nodes.append(new_node)
        self.kdtree_needs_update = True

    def update_start(self,pos):
        self.goal = pos

    def sample(self):
        if self.goal_reached:
            return self.informed_sample()
        if random.random() < self.explore_bias:
            return self.goal
        else:
            return np.random.uniform(-20, 20, size=2)

    def informed_sample(self):
        c_best = self.path_length()
        c_min = np.linalg.norm(self.goal - self.start.position)
        if c_best == float('inf'):
            return np.random.uniform(-20, 20, size=2)

        center = (self.start.position + self.goal) / 2
        a1 = (self.goal - self.start.position) / c_min
        theta = np.arctan2(a1[1], a1[0])
        rot = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])
        r1 = c_best / 2
        r2 = np.sqrt(c_best ** 2 - c_min ** 2) / 2 if c_best > c_min else 0.1
        sample = np.random.randn(2) * [r1, r2]
        return np.dot(rot, sample) + center

    def get_nearest(self, point):
        if self.kdtree_needs_update:
            self.kdtree = KDTree([n.position for n in self.nodes])
            self.kdtree_needs_update = False
        dist, idx = self.kdtree.query(point)
        return self.nodes[idx]

    def get_neighbors(self, point, radius):
        if self.kdtree_needs_update:
            self.kdtree = KDTree([n.position for n in self.nodes])
            self.kdtree_needs_update = False
        idxs = self.kdtree.query_ball_point(point, radius)
        return [self.nodes[i] for i in idxs]

    def is_collision_free(self, p1, p2):
        for poly in self.obstacles:
            for i in range(len(poly)):
                q1, q2 = np.array(poly[i]), np.array(poly[(i + 1) % len(poly)])
                if self.segment_intersect(p1, p2, q1, q2):
                    return False
        return True

    def in_obstacle(self, point):
        for poly in self.obstacles:
            if self.point_in_polygon(point, poly):
                return True
        return False

    def check_goal(self):
        # if self.goal_reached:
        #     return

        for node in self.nodes:
            if np.linalg.norm(node.position - self.goal) <= self.goal_radius:
                self.goal_reached = True
                self.construct_path(node)
                break

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

    @staticmethod
    def point_in_polygon(point, poly):
        x, y = point
        inside = False
        n = len(poly)
        px1, py1 = poly[0]
        for i in range(n + 1):
            px2, py2 = poly[i % n]
            if min(py1, py2) < y <= max(py1, py2):
                if x <= max(px1, px2):
                    if py1 != py2:
                        xinters = (y - py1) * (px2 - px1) / (py2 - py1) + px1
                    if px1 == px2 or x <= xinters:
                        inside = not inside
            px1, py1 = px2, py2
        return inside

    @staticmethod
    def segment_intersect(p1, p2, q1, q2):
        def ccw(a, b, c):
            return (c[1] - a[1]) * (b[0] - a[0]) > (b[1] - a[1]) * (c[0] - a[0])
        return ccw(p1, q1, q2) != ccw(p2, q1, q2) and ccw(p1, p2, q1) != ccw(p1, p2, q2)
