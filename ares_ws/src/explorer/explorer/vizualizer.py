import cv2
import numpy as np

class RRTVisualizer:
    def __init__(self, map_array, resolution=0.05, origin=(0.0, 0.0), window_name='RRT Debug'):
        self.map_array = map_array
        self.resolution = resolution
        self.origin = origin
        self.window_name = window_name
        self.map_img = self._map_to_img()
        self.scale = 2  # Scale up for visibility

    def _map_to_img(self):
        img = np.full_like(self.map_array, 255, dtype=np.uint8)
        img[self.map_array == 100] = 0   # Obstacle
        img[self.map_array == -1] = 128  # Unknown
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)

    def world_to_map(self, pos):
        x = int((pos[0] - self.origin[0]) / self.resolution)
        y = int((pos[1] - self.origin[1]) / self.resolution)
        return (x, self.map_array.shape[0] - y - 1)

    def draw(self, robot_pos, goal_pos, nodes, path):
        img = self.map_img.copy()
        for node in nodes:
            if node.parent:
                a = self.world_to_map(node.position)
                b = self.world_to_map(node.parent.position)
                cv2.line(img, a, b, (200, 200, 200), 1)

        for pt in path:
            mp = self.world_to_map(pt)
            cv2.circle(img, mp, 2, (0, 255, 0), -1)

        cv2.circle(img, self.world_to_map(robot_pos), 3, (255, 0, 0), -1)
        cv2.circle(img, self.world_to_map(goal_pos), 3, (0, 0, 255), -1)

        img = cv2.resize(img, None, fx=self.scale, fy=self.scale, interpolation=cv2.INTER_NEAREST)
        cv2.imshow(self.window_name, img)
        cv2.waitKey(1)
