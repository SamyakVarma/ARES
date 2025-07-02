import pygame
import numpy as np
import random
import math
import time

# Constants
WIDTH, HEIGHT = 800, 600
MAP_RESOLUTION = 10
ROWS, COLS = HEIGHT // MAP_RESOLUTION, WIDTH // MAP_RESOLUTION

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 150, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
PURPLE = (200, 0, 200)
YELLOW = (255, 255, 0)

CLEARANCE = 10  # pixels clearance around obstacles
RADIUS = 40    # neighborhood radius for rewiring
STEP_SIZE = 20
GOAL_BIAS = 0.1
PRUNE_DELAY = 2.0  # seconds to wait before pruning nodes stuck in obstacles

random.seed(24)
np.random.seed(43)

pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Informed RRT* with Dynamic Obstacles & Delayed Pruning")
clock = pygame.time.Clock()

# Generate static map with random obstacles
def generate_map():
    grid = np.full((ROWS, COLS), 255, dtype=np.uint8)
    for _ in range(100):
        r, c = random.randint(0, ROWS - 3), random.randint(0, COLS - 3)
        h, w = random.randint(1, 2), random.randint(1, 2)
        grid[r:r+h, c:c+w] = 0
    return grid

def inflate_obstacles(grid, clearance_pixels):
    inflated = grid.copy()
    kernel_radius = int(clearance_pixels // MAP_RESOLUTION)
    for r in range(ROWS):
        for c in range(COLS):
            if grid[r, c] == 0:
                for dr in range(-kernel_radius, kernel_radius + 1):
                    for dc in range(-kernel_radius, kernel_radius + 1):
                        rr, cc = r + dr, c + dc
                        if 0 <= rr < ROWS and 0 <= cc < COLS:
                            if math.hypot(dr, dc) * MAP_RESOLUTION <= clearance_pixels:
                                inflated[rr, cc] = 0
    return inflated

def draw_map(grid_real, grid_inflated):
    screen.fill(WHITE)
    inflated_surface = pygame.Surface((WIDTH, HEIGHT), pygame.SRCALPHA)
    for r in range(ROWS):
        for c in range(COLS):
            x, y = c * MAP_RESOLUTION, r * MAP_RESOLUTION
            if grid_real[r, c] == 0:
                pygame.draw.rect(screen, BLACK, (x, y, MAP_RESOLUTION, MAP_RESOLUTION))
            if grid_inflated[r, c] == 0 and grid_real[r, c] != 0:
                # Transparent inflated clearance (e.g., blue with alpha)
                pygame.draw.rect(inflated_surface, (0, 0, 255,80), (x, y, MAP_RESOLUTION, MAP_RESOLUTION))
    screen.blit(inflated_surface, (0, 0))

class DynamicObstacle:
    def __init__(self, path, radius=15, speed=2.0):
        self.path = path
        self.radius = radius
        self.speed = speed
        self.index = 0
        self.pos = path[0]

    def update(self):
        target = self.path[self.index]
        dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        dist = math.hypot(dx, dy)
        if dist < self.speed:
            self.index = (self.index + 1) % len(self.path)
            target = self.path[self.index]
            dx, dy = target[0] - self.pos[0], target[1] - self.pos[1]
        angle = math.atan2(dy, dx)
        self.pos = (
            self.pos[0] + self.speed * math.cos(angle),
            self.pos[1] + self.speed * math.sin(angle)
        )

    def draw(self):
        pygame.draw.circle(screen, PURPLE, (int(self.pos[0]), int(self.pos[1])), self.radius)

    def occupy_grid(self, grid):
        cx, cy = int(self.pos[0] // MAP_RESOLUTION), int(self.pos[1] // MAP_RESOLUTION)
        for r in range(ROWS):
            for c in range(COLS):
                if 0 <= r < ROWS and 0 <= c < COLS:
                    if math.hypot(c - cx, r - cy) * MAP_RESOLUTION <= self.radius:
                        grid[r, c] = 0

class Node:
    def __init__(self, pos):
        self.pos = pos
        self.parent = None
        self.cost = 0
        self.in_obstacle_since = None  # timestamp when node first detected in obstacle

def distance(p1, p2):
    return math.hypot(p1[0] - p2[0], p1[1] - p2[1])

def is_collision_free(grid, p1, p2):
    # Bresenham or interpolated segment check
    steps = int(distance(p1, p2) / 2)
    for i in range(steps + 1):
        t = i / steps if steps > 0 else 0
        x = int((p1[0] * (1 - t) + p2[0] * t) / MAP_RESOLUTION)
        y = int((p1[1] * (1 - t) + p2[1] * t) / MAP_RESOLUTION)
        if 0 <= y < ROWS and 0 <= x < COLS:
            if grid[y, x] == 0:
                return False
        else:
            return False
    return True

def get_nearest(nodes, pt):
    return min(nodes, key=lambda n: distance(n.pos, pt))

def get_nearby(nodes, pt, radius):
    return [n for n in nodes if distance(n.pos, pt) < radius]

def steer(from_node, to_pos):
    d = distance(from_node.pos, to_pos)
    if d < STEP_SIZE:
        return to_pos
    theta = math.atan2(to_pos[1] - from_node.pos[1], to_pos[0] - from_node.pos[0])
    return (
        from_node.pos[0] + STEP_SIZE * math.cos(theta),
        from_node.pos[1] + STEP_SIZE * math.sin(theta)
    )

def sample_with_goal_bias(goal):
    if random.random() < GOAL_BIAS:
        return (int(np.clip(np.random.normal(goal[0], 40), 0, WIDTH)), int(np.clip(np.random.normal(goal[1], 40), 0, HEIGHT)))
    else:
        return (random.randint(0, WIDTH), random.randint(0, HEIGHT))

def sample_in_ellipse(start, goal, c_best):
    if c_best == float('inf'):
        return sample_with_goal_bias(goal)
    center = ((start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2)
    c_min = distance(start, goal)
    if c_best < c_min:
        c_best = c_min + 1e-6
    a = c_best / 2
    b = math.sqrt(a**2 - (c_min / 2)**2)
    while True:
        x, y = random.uniform(-1, 1), random.uniform(-1, 1)
        if x**2 + y**2 <= 1:
            break
    theta = math.atan2(goal[1] - start[1], goal[0] - start[0])
    rot = np.array([[math.cos(theta), -math.sin(theta)], [math.sin(theta), math.cos(theta)]])
    sample = np.dot(rot, np.array([x * a, y * b])) + np.array(center)
    return int(np.clip(sample[0], 0, WIDTH)), int(np.clip(sample[1], 0, HEIGHT))

def extract_path(goal_node):
    path = []
    curr = goal_node
    while curr:
        path.append(curr.pos)
        curr = curr.parent
    return list(reversed(path))

def draw_path(path):
    for i in range(1, len(path)):
        pygame.draw.line(screen, RED, path[i - 1], path[i], 3)

def path_blocked(path, grid):
    for pos in path:
        r, c = int(pos[1]) // MAP_RESOLUTION, int(pos[0]) // MAP_RESOLUTION
        if r < 0 or r >= ROWS or c < 0 or c >= COLS or grid[r, c] == 0:
            return True
    return False

def follow_path(bot_pos, path, speed):
    if not path or len(path) < 2:
        return bot_pos
    for i in range(len(path) - 1):
        seg_start, seg_end = path[i], path[i + 1]
        seg_vec = np.array(seg_end) - np.array(seg_start)
        seg_len = np.linalg.norm(seg_vec)
        to_bot_vec = np.array(bot_pos) - np.array(seg_start)
        proj_len = np.dot(to_bot_vec, seg_vec) / (seg_len + 1e-6)
        if 0 <= proj_len < seg_len:
            direction = seg_vec / (seg_len + 1e-6)
            return tuple(np.array(bot_pos) + speed * direction)
    return path[-1]

def main():
    static_grid = generate_map()
    start = (50, 50)
    goal = (750, 550)
    bot_pos = start
    BOT_SPEED = 2.0
    sampled_points = []

    dynamic_obs = [
        DynamicObstacle([(200, 100), (200, 500)], speed=1.5),
        DynamicObstacle([(500, 100), (600, 500), (700, 100)], speed=2.0)
    ]

    nodes = [Node(bot_pos)]
    best_goal_node = None
    best_cost = float('inf')

    static_obstacle_added = False
    add_obstacle_after = 10  # seconds
    start_time = time.time()

    running = True
    while running:
        current_time = time.time()
        elapsed = current_time - start_time

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        if not static_obstacle_added and elapsed > add_obstacle_after:
            r_start, c_start = ROWS//2 - 5, COLS//2 - 10
            r_end, c_end = ROWS//2 + 5, COLS//2 + 10
            for r in range(r_start, r_end):
                for c in range(c_start, c_end):
                    static_grid[r, c] = 0
            static_obstacle_added = True
            print("Static obstacle added!")

        for obs in dynamic_obs:
            obs.update()

        # Prepare occupancy grid
        grid_real = static_grid.copy()
        grid_inflated = inflate_obstacles(grid_real, CLEARANCE)

        for obs in dynamic_obs:
            obs.occupy_grid(grid_inflated)  # For planning
            obs.occupy_grid(grid_real)      # For real collision

        # Soft re-root: Keep existing tree, add new root from bot_pos
        # Always add bot's current position as a new root node
        new_root = Node(bot_pos)

        # Try to connect to the existing tree
        nearest = get_nearest(nodes, bot_pos)
        if is_collision_free(grid_inflated, bot_pos, nearest.pos):
            new_root.parent = nearest
            new_root.cost = nearest.cost + distance(bot_pos, nearest.pos)

        nodes.append(new_root)

        # Path replanning
        if best_goal_node:
            rand_pt = sample_in_ellipse(bot_pos, goal, best_cost)
        else:
            rand_pt = sample_with_goal_bias(goal)
        sampled_points.append(rand_pt)

        nearest = get_nearest(nodes, rand_pt)
        new_pos = steer(nearest, rand_pt)

        if not is_collision_free(grid_inflated, nearest.pos, new_pos):
            continue

        new_node = Node(new_pos)
        new_node.cost = nearest.cost + distance(nearest.pos, new_pos)
        new_node.parent = nearest

        for node in nodes[:]:
            r, c = int(node.pos[1]) // MAP_RESOLUTION, int(node.pos[0]) // MAP_RESOLUTION
            if 0 <= r < ROWS and 0 <= c < COLS and grid_inflated[r, c] == 0:
                if node.in_obstacle_since is None:
                    node.in_obstacle_since = current_time
                elif current_time - node.in_obstacle_since > PRUNE_DELAY:
                    def prune_recursive(n):
                        children = [nd for nd in nodes if nd.parent == n]
                        for child in children:
                            prune_recursive(child)
                        if n in nodes:
                            nodes.remove(n)
                    if best_goal_node and path_blocked(extract_path(best_goal_node), grid_real):
                        best_goal_node = None
                        best_cost = float('inf')
                    prune_recursive(node)
            else:
                node.in_obstacle_since = None

        near_nodes = get_nearby(nodes, new_pos, RADIUS)

        for node in near_nodes:
            if is_collision_free(grid_inflated, node.pos, new_pos):
                cost = node.cost + distance(node.pos, new_pos)
                if cost < new_node.cost:
                    new_node.parent = node
                    new_node.cost = cost

        nodes.append(new_node)

        for node in near_nodes:
            if node == new_node.parent:
                continue
            if is_collision_free(grid_inflated, new_node.pos, node.pos):
                new_cost = new_node.cost + distance(new_node.pos, node.pos)
                if new_cost < node.cost:
                    node.parent = new_node
                    node.cost = new_cost

        if distance(new_node.pos, goal) < STEP_SIZE and is_collision_free(grid_inflated, new_node.pos, goal):
            temp_goal = Node(goal)
            temp_goal.parent = new_node
            temp_goal.cost = new_node.cost + distance(new_node.pos, goal)
            nodes.append(temp_goal)
            if temp_goal.cost < best_cost:
                best_goal_node = temp_goal
                best_cost = temp_goal.cost

        # Move bot along best path
        if best_goal_node:
            path = extract_path(best_goal_node)
            if len(path) > 1:
                next_node = path[1]
                # Move bot towards next node
                bot_pos = follow_path(bot_pos, path, BOT_SPEED)
                # Check if bot reached or passed next node
                if distance(bot_pos, next_node) < BOT_SPEED:
                    # Re-root tree at current bot position to replan from here
                    nodes = [Node(bot_pos)]
                    best_goal_node = None
                    best_cost = float('inf')
            else:
                # Path only has one node (at goal), just move there
                bot_pos = follow_path(bot_pos, path, BOT_SPEED)

        # Visualization
        screen.fill(BLACK)
        draw_map(grid_real, grid_inflated)
        for obs in dynamic_obs:
            obs.draw()
        pygame.draw.circle(screen, GREEN, (int(bot_pos[0]), int(bot_pos[1])), 5)
        pygame.draw.circle(screen, RED, goal, 5)

        for pt in sampled_points[-300:]:  # Show only the latest 300 to avoid clutter
            pygame.draw.circle(screen, YELLOW, (int(pt[0]), int(pt[1])), 2)
            
        for node in nodes:
            if node.parent:
                pygame.draw.line(screen, BLUE, node.pos, node.parent.pos, 1)

        if best_goal_node:
            draw_path(extract_path(best_goal_node))

        pygame.display.update()
        clock.tick(60)

    pygame.quit()

if __name__ == "__main__":
    main()