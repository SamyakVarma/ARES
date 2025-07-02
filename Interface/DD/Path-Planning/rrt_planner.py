import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import KDTree
import random

class Node:
    def __init__(self, position, parent=None, cost=0.0):
        self.position = np.array(position)
        self.parent = parent
        self.children = []
        self.cost = cost  # For RRT*

class RRTPlanner:
    def __init__(self, start, goal, goal_radius, obstacles, tree_type="RRT*", explore_bias=0.05):
        self.start = Node(start)
        self.goal = np.array(goal)
        self.goal_radius = goal_radius
        self.obstacles = obstacles
        self.tree_type = tree_type
        self.explore_bias = explore_bias
        self.nodes = [self.start]
        self.goal_reached = False
        self.path = []
        self.rrt_step = 1.0
        self.rrt_star_radius = 3.0

    def plan(self, max_nodes):
        for _ in range(max_nodes):
            self.add_node()
            if not self.goal_reached:
                self.check_goal()

    def add_node(self):
        # Sample point
        if random.random() < self.explore_bias:
            rnd_point = self.goal
        else:
            rnd_point = np.round(np.random.uniform(-20, 20, size=2), 2)

        # Find nearest node
        positions = np.array([node.position for node in self.nodes])
        tree = KDTree(positions)
        if self.tree_type in ("RRT", "RRT*"):
            _, idx = tree.query(rnd_point)
        else:
            idx = random.randint(0, len(self.nodes) - 1)
        nearest = self.nodes[idx]

        direction = rnd_point - nearest.position
        norm = np.linalg.norm(direction)
        if norm == 0:
            return
        direction = direction / norm
        new_pos = nearest.position + direction * min(self.rrt_step, norm)

        if self.in_collision(nearest.position, new_pos):
            return

        # Create and add new node
        new_node = Node(new_pos, parent=nearest)
        new_node.cost = nearest.cost + np.linalg.norm(new_node.position - nearest.position)
        nearest.children.append(new_node)

        if self.tree_type == "RRT*":
            # Rewire nearby nodes
            neighbors_idx = tree.query_ball_point(new_pos, self.rrt_star_radius)
            for i in neighbors_idx:
                neighbor = self.nodes[i]
                dist = np.linalg.norm(new_node.position - neighbor.position)
                new_cost = new_node.cost + dist
                if not self.in_collision(new_node.position, neighbor.position) and new_cost < neighbor.cost:
                    # Rewire
                    if neighbor.parent:
                        neighbor.parent.children.remove(neighbor)
                    neighbor.parent = new_node
                    neighbor.cost = new_cost
                    new_node.children.append(neighbor)
                    self.update_cost_to_go(neighbor)

        self.nodes.append(new_node)

    def update_cost_to_go(self, node):
        for child in node.children:
            child.cost = node.cost + np.linalg.norm(child.position - node.position)
            self.update_cost_to_go(child)

    def in_collision(self, p1, p2):
        for obs in self.obstacles:
            if self.segment_intersects_polygon(p1, p2, obs):
                return True
        return False

    def segment_intersects_polygon(self, p1, p2, polygon):
        for i in range(len(polygon)):
            q1 = polygon[i]
            q2 = polygon[(i + 1) % len(polygon)]
            if self.segments_intersect(p1, p2, q1, q2):
                return True
        return False

    def segments_intersect(self, a1, a2, b1, b2):
        def ccw(p1, p2, p3):
            return (p3[1] - p1[1]) * (p2[0] - p1[0]) > (p2[1] - p1[1]) * (p3[0] - p1[0])
        return ccw(a1, b1, b2) != ccw(a2, b1, b2) and ccw(a1, a2, b1) != ccw(a1, a2, b2)

    def check_goal(self):
        best_goal_node = None
        best_cost = float('inf')
        for node in self.nodes:
            if np.linalg.norm(node.position - self.goal) < self.goal_radius:
                if node.cost < best_cost:
                    best_goal_node = node
                    best_cost = node.cost

        if best_goal_node:
            self.goal_reached = True
            self.extract_path(best_goal_node)

    def extract_path(self, node):
        self.path = []
        while node is not None:
            self.path.append(node.position)
            node = node.parent
        self.path.reverse()

    def draw(self):
        fig, ax = plt.subplots(figsize=(6, 6))
        ax.set_xlim(-21, 21)
        ax.set_ylim(-21, 23)

        # Obstacles
        for poly in self.obstacles:
            patch = plt.Polygon(poly, color='blue', alpha=0.5)
            ax.add_patch(patch)

        # Goal
        goal_circle = plt.Circle(self.goal, self.goal_radius, color='green' if self.goal_reached else 'yellow')
        ax.add_patch(goal_circle)
        ax.plot(*self.goal, 'yo')

        # Tree
        for node in self.nodes[1:]:
            ax.plot([node.position[0], node.parent.position[0]], [node.position[1], node.parent.position[1]], 'k-', linewidth=0.5)

        # Path
        if self.goal_reached and self.path:
            path = np.array(self.path)
            ax.plot(path[:, 0], path[:, 1], 'g-', linewidth=2)

        plt.title(f"{len(self.nodes)} nodes | {'Path length: {:.2f}'.format(self.path_length()) if self.goal_reached else 'Goal not yet reached'}")
        plt.grid(True)
        plt.show()

    def path_length(self):
        return sum(np.linalg.norm(self.path[i] - self.path[i-1]) for i in range(1, len(self.path)))

