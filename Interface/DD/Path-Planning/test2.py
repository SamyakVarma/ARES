import pygame
import sys
import numpy as np
from rrtInf import RRTPlanner

# Constants
WIDTH, HEIGHT = 800, 800
SCALE = 20  # World units to screen pixels

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED   = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE  = (0, 0, 255)
GRAY  = (200, 200, 200)

# Convert world coordinates to screen
def world_to_screen(pos):
    return int(WIDTH // 2 + pos[0] * SCALE), int(HEIGHT // 2 - pos[1] * SCALE)

def draw_obstacles(screen, obstacles):
    for poly in obstacles:
        pts = [world_to_screen(p) for p in poly]
        pygame.draw.polygon(screen, BLACK, pts, width=0)

def draw_tree(screen, nodes):
    for node in nodes:
        if node.parent is not None:
            p1 = world_to_screen(node.position)
            p2 = world_to_screen(node.parent.position)
            pygame.draw.line(screen, BLUE, p1, p2, width=1)

def draw_path(screen, path):
    if len(path) < 2:
        return
    pts = [world_to_screen(p) for p in path]
    pygame.draw.lines(screen, RED, False, pts, width=3)

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("RRT* with Informed Sampling")
    clock = pygame.time.Clock()

    start = (-15, -15)
    goal = None
    obstacles = [
        [(-5, -5), (-3, -5), (-3, 5), (-5, 5)],
        [(3, -5), (5, -5), (5, 5), (3, 5)]
    ]

    planner = None
    running = True
    planning_started = False

    while running:
        screen.fill(WHITE)
        draw_obstacles(screen, obstacles)

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
                break
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                gx, gy = event.pos
                wx = (gx - WIDTH // 2) / SCALE
                wy = (HEIGHT // 2 - gy) / SCALE
                goal = (wx, wy)
                if not planning_started:
                    planner = RRTPlanner(start, goal, goal_radius=1.5, obstacles=obstacles, step_size=1.0)
                else:
                    planner.update_start(goal)
                planning_started = True

        if planner and planning_started and len(planner.nodes) < planner.max_nodes:
            for _ in range(10):
                planner.add_node()
                planner.check_goal()

        if planner:
            draw_tree(screen, planner.nodes)
            draw_path(screen, planner.path)
            pygame.draw.circle(screen, GREEN, world_to_screen(start), 6)
            pygame.draw.circle(screen, RED, world_to_screen(goal), 6)

        pygame.display.flip()
        clock.tick(120)

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
