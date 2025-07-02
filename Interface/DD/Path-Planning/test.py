<<<<<<< HEAD
import numpy as np
import pygame
import sys
import cv2
from rrt_planner_new import RRTPlanner  # Ensure the planner is saved as rrt_planner_new.py

WIDTH, HEIGHT = 800, 800
GRID_SIZE = 400
CELL_SIZE = WIDTH // GRID_SIZE

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (120, 120, 120)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (50, 100, 255)
YELLOW = (255, 255, 0)

class DebugVisualizer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("RRT* with Sample + Ellipse Visualization")
        self.clock = pygame.time.Clock()

        self.grid = self.make_map()
        self.resolution = 1.0
        self.origin = np.array([0.0, 0.0])

        self.override_start = None
        self.override_goal = None
        self.planner = None
        self.last_sample = None

        self.grid_surface = pygame.Surface((WIDTH, HEIGHT))
        self.draw_map_once()

    def make_map(self):
        grid = np.ones((GRID_SIZE, GRID_SIZE), dtype=np.int8) * -1
        center = GRID_SIZE // 2
        cv2.circle(grid, (center, center), 120, 0, -1)
        cv2.rectangle(grid, (80, 100), (100, 300), 100, -1)
        cv2.rectangle(grid, (150, 200), (180, 250), 100, -1)
        cv2.rectangle(grid, (250, 100), (260, 300), 100, -1)
        return grid

    def draw_map_once(self):
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                v = self.grid[y, x]
                if v == 0:
                    color = WHITE
                elif v == 100:
                    color = BLACK
                else:
                    color = GRAY
                pygame.draw.rect(self.grid_surface, color, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    def screen_to_world(self, px, py):
        return np.array([px / CELL_SIZE, (HEIGHT - py) / CELL_SIZE])

    def world_to_screen(self, pos):
        return int(pos[0] * CELL_SIZE), HEIGHT - int(pos[1] * CELL_SIZE)

    def try_initialize_planner(self):
        if self.override_start is None or self.override_goal is None:
            return

        self.planner = RRTPlanner(
            start=self.override_start,
            goal=self.override_goal,
            grid_size=GRID_SIZE,
            goal_radius=3.0,
            occupancy_grid=self.grid,
            origin=self.origin,
            resolution=self.resolution,
            step_size=4.0,
            max_nodes=3000
        )
        print(f"[Planner] Start: {self.override_start}, Goal: {self.override_goal}")

    def draw_tree(self):
        if not self.planner:
            return
        for node in self.planner.nodes:
            if node.parent is not None:
                p1 = self.world_to_screen(node.position)
                p2 = self.world_to_screen(node.parent.position)
                pygame.draw.line(self.screen, BLUE, p1, p2, 1)

    def draw_path(self):
        if not self.planner or not self.planner.path:
            return
        pts = [self.world_to_screen(p) for p in self.planner.path]
        pygame.draw.lines(self.screen, RED, False, pts, 3)

    def draw_last_sample(self):
        if self.last_sample is not None:
            x, y = self.world_to_screen(self.last_sample)
            pygame.draw.circle(self.screen, YELLOW, (x, y), 3)

    def draw_ellipse(self):
        if not self.planner or not self.planner.goal_reached:
            return

        c_best = self.planner.path_length()
        c_min = np.linalg.norm(self.planner.goal - self.planner.start.position)
        if c_best == float('inf') or c_best <= c_min:
            return

        center = (self.planner.start.position + self.planner.goal) / 2
        a1 = (self.planner.goal - self.planner.start.position) / c_min
        theta = np.arctan2(a1[1], a1[0])

        # Semi-major and semi-minor
        r1 = c_best / 2
        r2 = np.sqrt(c_best ** 2 - c_min ** 2) / 2 if c_best > c_min else 0.1

        # Ensure r1 >= r2
        if r2 > r1:
            r1, r2 = r2, r1
            theta += np.pi / 2  # rotate by 90° if axes flipped

        angle_deg = -np.degrees(theta)

        # Create ellipse surface
        ellipse_w, ellipse_h = 2 * r1 * CELL_SIZE, 2 * r2 * CELL_SIZE
        ellipse_surface = pygame.Surface((ellipse_w, ellipse_h), pygame.SRCALPHA)
        pygame.draw.ellipse(ellipse_surface, (0, 0, 255, 50), (0, 0, ellipse_w, ellipse_h), 2)

        # Rotate surface
        rotated = pygame.transform.rotate(ellipse_surface, angle_deg)
        rotated_rect = rotated.get_rect()
        screen_center = self.world_to_screen(center)
        rotated_rect.center = screen_center

        self.screen.blit(rotated, rotated_rect)

    def run(self):
        while True:
            self.screen.blit(self.grid_surface, (0, 0))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    world = self.screen_to_world(*pygame.mouse.get_pos())
                    if event.button == 1:
                        self.override_goal = world
                    elif event.button == 3:
                        self.override_start = world
                    self.try_initialize_planner()

            if self.planner:
                for _ in range(10):
                    sample = self.planner.sample(GRID_SIZE)
                    self.last_sample = sample
                    nearest = self.planner.get_nearest(sample)
                    direction = sample - nearest.position
                    length = np.linalg.norm(direction)
                    if length > 0:
                        new_pos = nearest.position + self.planner.step_size * direction / length
                        if not self.planner.in_obstacle(new_pos):
                            self.planner.add_node()
                            self.planner.check_goal()

                self.draw_tree()
                self.draw_path()
                self.draw_ellipse()
                self.draw_last_sample()

            if self.override_start is not None:
                pygame.draw.circle(self.screen, GREEN, self.world_to_screen(self.override_start), 6)

            if self.override_goal is not None:
                pygame.draw.circle(self.screen, RED, self.world_to_screen(self.override_goal), 6)

            pygame.display.flip()
            self.clock.tick(30)


if __name__ == "__main__":
    DebugVisualizer().run()
=======
import numpy as np
import pygame
import sys
import cv2
from rrt_planner_new import RRTPlanner  # Ensure the planner is saved as rrt_planner_new.py

WIDTH, HEIGHT = 800, 800
GRID_SIZE = 400
CELL_SIZE = WIDTH // GRID_SIZE

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (120, 120, 120)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (50, 100, 255)
YELLOW = (255, 255, 0)

class DebugVisualizer:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        pygame.display.set_caption("RRT* with Sample + Ellipse Visualization")
        self.clock = pygame.time.Clock()

        self.grid = self.make_map()
        self.resolution = 1.0
        self.origin = np.array([0.0, 0.0])

        self.override_start = None
        self.override_goal = None
        self.planner = None
        self.last_sample = None

        self.grid_surface = pygame.Surface((WIDTH, HEIGHT))
        self.draw_map_once()

    def make_map(self):
        grid = np.ones((GRID_SIZE, GRID_SIZE), dtype=np.int8) * -1
        center = GRID_SIZE // 2
        cv2.circle(grid, (center, center), 120, 0, -1)
        cv2.rectangle(grid, (80, 100), (100, 300), 100, -1)
        cv2.rectangle(grid, (150, 200), (180, 250), 100, -1)
        cv2.rectangle(grid, (250, 100), (260, 300), 100, -1)
        return grid

    def draw_map_once(self):
        for y in range(GRID_SIZE):
            for x in range(GRID_SIZE):
                v = self.grid[y, x]
                if v == 0:
                    color = WHITE
                elif v == 100:
                    color = BLACK
                else:
                    color = GRAY
                pygame.draw.rect(self.grid_surface, color, (x * CELL_SIZE, y * CELL_SIZE, CELL_SIZE, CELL_SIZE))

    def screen_to_world(self, px, py):
        return np.array([px / CELL_SIZE, (HEIGHT - py) / CELL_SIZE])

    def world_to_screen(self, pos):
        return int(pos[0] * CELL_SIZE), HEIGHT - int(pos[1] * CELL_SIZE)

    def try_initialize_planner(self):
        if self.override_start is None or self.override_goal is None:
            return

        self.planner = RRTPlanner(
            start=self.override_start,
            goal=self.override_goal,
            grid_size=GRID_SIZE,
            goal_radius=3.0,
            occupancy_grid=self.grid,
            origin=self.origin,
            resolution=self.resolution,
            step_size=4.0,
            max_nodes=3000
        )
        print(f"[Planner] Start: {self.override_start}, Goal: {self.override_goal}")

    def draw_tree(self):
        if not self.planner:
            return
        for node in self.planner.nodes:
            if node.parent is not None:
                p1 = self.world_to_screen(node.position)
                p2 = self.world_to_screen(node.parent.position)
                pygame.draw.line(self.screen, BLUE, p1, p2, 1)

    def draw_path(self):
        if not self.planner or not self.planner.path:
            return
        pts = [self.world_to_screen(p) for p in self.planner.path]
        pygame.draw.lines(self.screen, RED, False, pts, 3)

    def draw_last_sample(self):
        if self.last_sample is not None:
            x, y = self.world_to_screen(self.last_sample)
            pygame.draw.circle(self.screen, YELLOW, (x, y), 3)

    def draw_ellipse(self):
        if not self.planner or not self.planner.goal_reached:
            return

        c_best = self.planner.path_length()
        c_min = np.linalg.norm(self.planner.goal - self.planner.start.position)
        if c_best == float('inf') or c_best <= c_min:
            return

        center = (self.planner.start.position + self.planner.goal) / 2
        a1 = (self.planner.goal - self.planner.start.position) / c_min
        theta = np.arctan2(a1[1], a1[0])

        # Semi-major and semi-minor
        r1 = c_best / 2
        r2 = np.sqrt(c_best ** 2 - c_min ** 2) / 2 if c_best > c_min else 0.1

        # Ensure r1 >= r2
        if r2 > r1:
            r1, r2 = r2, r1
            theta += np.pi / 2  # rotate by 90° if axes flipped

        angle_deg = -np.degrees(theta)

        # Create ellipse surface
        ellipse_w, ellipse_h = 2 * r1 * CELL_SIZE, 2 * r2 * CELL_SIZE
        ellipse_surface = pygame.Surface((ellipse_w, ellipse_h), pygame.SRCALPHA)
        pygame.draw.ellipse(ellipse_surface, (0, 0, 255, 50), (0, 0, ellipse_w, ellipse_h), 2)

        # Rotate surface
        rotated = pygame.transform.rotate(ellipse_surface, angle_deg)
        rotated_rect = rotated.get_rect()
        screen_center = self.world_to_screen(center)
        rotated_rect.center = screen_center

        self.screen.blit(rotated, rotated_rect)

    def run(self):
        while True:
            self.screen.blit(self.grid_surface, (0, 0))

            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
                elif event.type == pygame.MOUSEBUTTONDOWN:
                    world = self.screen_to_world(*pygame.mouse.get_pos())
                    if event.button == 1:
                        self.override_goal = world
                    elif event.button == 3:
                        self.override_start = world
                    self.try_initialize_planner()

            if self.planner:
                for _ in range(10):
                    sample = self.planner.sample(GRID_SIZE)
                    self.last_sample = sample
                    nearest = self.planner.get_nearest(sample)
                    direction = sample - nearest.position
                    length = np.linalg.norm(direction)
                    if length > 0:
                        new_pos = nearest.position + self.planner.step_size * direction / length
                        if not self.planner.in_obstacle(new_pos):
                            self.planner.add_node()
                            self.planner.check_goal()

                self.draw_tree()
                self.draw_path()
                self.draw_ellipse()
                self.draw_last_sample()

            if self.override_start is not None:
                pygame.draw.circle(self.screen, GREEN, self.world_to_screen(self.override_start), 6)

            if self.override_goal is not None:
                pygame.draw.circle(self.screen, RED, self.world_to_screen(self.override_goal), 6)

            pygame.display.flip()
            self.clock.tick(30)


if __name__ == "__main__":
    DebugVisualizer().run()
>>>>>>> f0e7726e (updated face recognitions)
